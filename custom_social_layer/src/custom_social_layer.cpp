#include <custom_social_layer/custom_social_layer.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>
#include <stdio.h>
#include <custom_social_layer/People.h>
#include <custom_social_layer/Person.h>
#include <custom_social_layer/PeopleParametersConfig.h>

//Make the layer available as a ROS plugin
PLUGINLIB_EXPORT_CLASS(social_layer_namespace::SocialLayer, costmap_2d::Layer)

    using costmap_2d::NO_INFORMATION;
    using costmap_2d::LETHAL_OBSTACLE;

    //Here is where all the helper functions be written
    namespace social_layer_namespace
{
    SocialLayer::SocialLayer() {}
    void SocialLayer::onInitialize()
    {

        ros::NodeHandle nh("~/"+name_);
        current_ = true;

        //Subscribe to the people_messages topic
        default_value_ = NO_INFORMATION;
        people_sub_ = n.subscribe("/people_messages", 10, &SocialLayer::peopleCallback, this);

        /*Set up dynamic reconfigure to enable/disable layer as well as adjust
          personal space parameters*/

        dsrv_ = new dynamic_reconfigure::Server<custom_social_layer::PeopleParametersConfig> (nh);
        dynamic_reconfigure::Server<custom_social_layer::PeopleParametersConfig>::CallbackType
            f = boost::bind(&SocialLayer::reconfigure_people_params, this, _1, _2);
        dsrv_->setCallback(f);

    }

    //Update the list of people when possible
    void SocialLayer::peopleCallback(custom_social_layer::People people_list)
    {
        if (people_vector_.size() >= 1)
        {
            people_vector_.pop_back();
        }
        people_vector_.push_back(people_list);
    }


    void SocialLayer::reconfigure_people_params(custom_social_layer::PeopleParametersConfig &config, uint32_t level)
    {
        Costmap2D* master = layered_costmap_->getCostmap();
        double conversion_factor = 1/(master->getResolution());

        ellipse_height_ = config.ellipse_height * conversion_factor;
        ellipse_width_ = config.ellipse_width * conversion_factor;
        stretch_factor_ = config.stretch_factor;

    }

    //Matches the master grid's resolution to the costmap at hand
    void SocialLayer::matchSize()
    {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
    }


    void SocialLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_){
            return;
        }
        double personX, personY,  minX , minY, maxX, maxY;

        int len = people_vector_.size();

        /*The following loop extracts the min and max x and y values from
          the list of people poses */
        for(int i = 0; i < len; i++)
        {
            for(int j = 0; j < people_vector_[i].people.size(); j++)
            {
                if(j == 0)
                {
                    minX = people_vector_[i].people[j].pose.x;
                    maxX = people_vector_[i].people[j].pose.x;
                    minY = people_vector_[i].people[j].pose.x;
                    maxY = people_vector_[i].people[j].pose.x;
                }
                if(minX > people_vector_[i].people[j].pose.x)
                {
                    minX = people_vector_[i].people[j].pose.x;
                }
                if(minY > people_vector_[i].people[j].pose.y)
                {
                    minY = people_vector_[i].people[j].pose.y;
                }
                if(maxX < people_vector_[i].people[j].pose.x)
                {
                    maxX = people_vector_[i].people[j].pose.x;
                }
                if(maxY < people_vector_[i].people[j].pose.y)
                {
                    maxY = people_vector_[i].people[j].pose.y;
                }

            }
        }

        /*The following updates the min and max x, y values to be updated on the map
          using the min and max from the previous loop and the personal space params*/

        *min_x = std::min(*min_x, minX - ellipse_width_);
        *min_y = std::min(*min_y, minY - ellipse_height_);
        *max_x = std::max(*max_x, maxX + ellipse_width_);
        *max_y = std::max(*max_y, maxY + ellipse_height_);

    }

    void SocialLayer::updateCosts(costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_) return;

        int ellipse_height;
        int len = people_vector_.size();
        for(int i = 0; i < len; i++)
        {
            int len2 = people_vector_[i].people.size();

            for(int k = 0; k < len2; k++)
            {
                //update ellipse height based on velocity and stretch factor
                ellipse_height = ellipse_height_ + stretch_factor_ * people_vector_[i].people[k].velocity;

                //Call ellipse drawing helper function
                midptellipse(master_grid, ellipse_width_/2, ellipse_height/2,
                        people_vector_[i].people[k].pose.x,
                        people_vector_[i].people[k].pose.y,
                        people_vector_[i].people[k].pose.theta);
            }
        }

    }

    /*The following is an implementation of Bresenham's midpoint ellipse drawing algorithm with appropriate
      modifications to construct personal space around people*/

    void SocialLayer::midptellipse(costmap_2d::Costmap2D & master_grid, int rx, int ry, int xOrigin, int yOrigin, double orientation)
    {
        float dx, dy,d1, d2, x, y, x1, x2, y1, y2, xFloat1, yFloat1, xFloat2, yFloat2, xFloat3, yFloat3, xFloat4, yFloat4;
        int xPos1, xPos2, xPos3, xPos4, yPos1, yPos2, yPos3, yPos4;
        x = 0;
        y = ry; //y = y radius

        d1 = (ry*ry) - (rx *rx * ry) + (0.25*rx*rx);
        dx = 2 * rx * ry * x;
        dy = 2 * rx * rx * y;

        while (dx < dy){

            x1 = x+xOrigin;
            y1 = y+yOrigin;
            x2 = -x+xOrigin;
            y2 = -y+yOrigin;

            for (int i = (int)-x+xOrigin; i <= (int)x+xOrigin; i++){
                double iDouble = i * 1.0;
                double xDouble = (i - xOrigin) * cos(orientation) + (y1-yOrigin)*sin(orientation);
                xDouble = xDouble + xOrigin;
                double yDouble = (y1 - yOrigin) * cos(orientation) - (i - xOrigin) * sin (orientation);
                yDouble = yDouble + yOrigin;
                master_grid.setCost((int)round(xDouble), (int)round(yDouble), LETHAL_OBSTACLE);
            }

            /*The following lines allow us to rotate the shape at an angle dependent on the person's
              orientation about it's own axis*/
            xFloat1 = (x1-xOrigin) * cos(orientation) + (y1-yOrigin) * sin(orientation);
            xFloat1 = xFloat1 + xOrigin;
            yFloat1 = (y1-yOrigin) * cos(orientation) - (x1-xOrigin) * sin(orientation);
            yFloat1 = yFloat1+yOrigin;
            xPos1 = (int)round(xFloat1);
            yPos1 = (int)round(yFloat1);

            xFloat2 = (x2-xOrigin) * cos(orientation) + (y1-yOrigin) * sin(orientation);
            yFloat2 = (y1-yOrigin) * cos(orientation) - (x2-xOrigin) * sin(orientation);
            xFloat2 = xFloat2 + xOrigin;
            yFloat2 = yFloat2+yOrigin;
            xPos2 = (int)round(xFloat2);
            yPos2 = (int)round(yFloat2);

            xFloat3 = (x1-xOrigin) * cos(orientation) + (y2-yOrigin) * sin(orientation);
            yFloat3 = (y2-yOrigin) * cos(orientation) - (x1-xOrigin) * sin(orientation);
            xFloat3 = xFloat3 + xOrigin;
            yFloat3 = yFloat3+yOrigin;
            xPos3 = (int)round(xFloat3);
            yPos3 = (int)round(yFloat3);

            xFloat4 = (x2-xOrigin) * cos(orientation) + (y2-yOrigin) * sin(orientation);
            yFloat4 = (y2-yOrigin) * cos(orientation) - (x2-xOrigin) * sin(orientation);
            xFloat4 = xFloat4 + xOrigin;
            yFloat4 = yFloat4+yOrigin;
            xPos4 = (int)round(xFloat4);
            yPos4 = (int)round(yFloat4);


            //Anti Aliased Cost Setting (drawing pixels around the four corners of a pixel)
            master_grid.setCost(xPos1, yPos1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1+1, yPos1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1, yPos1+1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1+1, yPos1+1, LETHAL_OBSTACLE);

            master_grid.setCost(xPos2, yPos2, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2+1, yPos2, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2, yPos2+1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2+1, yPos2+1, LETHAL_OBSTACLE);


            if(d1 < 0){
                x++;
                dx = dx + (2*ry*ry);
                d1 = d1 + dx + (ry*ry);
            }
            else{
                x++;
                y--;
                dx = dx + (2 * ry * ry);
                dy = dy - (2 * rx * rx);
                d1 = d1 + dx - dy + (ry * ry);
            }
        }
        d2 = ((ry * ry) * ((x + 0.5) * (x + 0.5))) +((rx * rx) * ((y - 1) * (y - 1))) -(rx * rx * ry * ry);
        while (y >= 0){

            x1 = x+xOrigin;
            y1 = y+yOrigin;
            x2 = -x+xOrigin;
            y2 = -y+yOrigin;

            for (int i = (int)-x+xOrigin; i <= (int)x+xOrigin; i++){
                double iDouble = i * 1.0;
                double xDouble = (i - xOrigin) * cos(orientation) + (y1-yOrigin)*sin(orientation);
                xDouble = xDouble + xOrigin;
                double yDouble = (y1 - yOrigin) * cos(orientation) - (i - xOrigin) * sin (orientation);
                yDouble = yDouble + yOrigin;
                master_grid.setCost((int)round(xDouble), (int)round(yDouble), LETHAL_OBSTACLE);
            }

            xFloat1 = (x1-xOrigin) * cos(orientation) + (y1-yOrigin) * sin(orientation);
            xFloat1 = xFloat1 + xOrigin;
            yFloat1 = (y1-yOrigin) * cos(orientation) - (x1-xOrigin) * sin(orientation);
            yFloat1 = yFloat1+yOrigin;
            xPos1 = (int)round(xFloat1);
            yPos1 = (int)round(yFloat1);

            xFloat2 = (x2-xOrigin) * cos(orientation) + (y1-yOrigin) * sin(orientation);
            yFloat2 = (y1-yOrigin) * cos(orientation) - (x2-xOrigin) * sin(orientation);
            xFloat2 = xFloat2 + xOrigin;
            yFloat2 = yFloat2+yOrigin;
            xPos2 = (int)round(xFloat2);
            yPos2 = (int)round(yFloat2);

            xFloat3 = (x1-xOrigin) * cos(orientation) + (y2-yOrigin) * sin(orientation);
            yFloat3 = (y2-yOrigin) * cos(orientation) - (x1-xOrigin) * sin(orientation);
            xFloat3 = xFloat3 + xOrigin;
            yFloat3 = yFloat3+yOrigin;
            xPos3 = (int)round(xFloat3);
            yPos3 = (int)round(yFloat3);

            xFloat4 = (x2-xOrigin) * cos(orientation) + (y2-yOrigin) * sin(orientation);
            yFloat4 = (y2-yOrigin) * cos(orientation) - (x2-xOrigin) * sin(orientation);
            xFloat4 = xFloat4 + xOrigin;
            yFloat4 = yFloat4+yOrigin;
            xPos4 = (int)round(xFloat4);
            yPos4 = (int)round(yFloat4);

            master_grid.setCost(xPos1, yPos1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1+1, yPos1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1, yPos1+1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos1+1, yPos1+1, LETHAL_OBSTACLE);

            master_grid.setCost(xPos2, yPos2, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2+1, yPos2, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2, yPos2+1, LETHAL_OBSTACLE);
            master_grid.setCost(xPos2+1, yPos2+1, LETHAL_OBSTACLE);

            if(d2 > 0){
                y--;
                dy = dy - (2*rx*rx);
                d2 = d2 + (rx * rx) - dy;
            }

            else{
                x++;
                y--;
                dx = dx + (2*ry*ry);
                dy = dy - (2 * rx * rx);
                d2 = d2 + dx - dy + (rx * rx);
            }
        }
    }

    SocialLayer::Coordinates SocialLayer::shear(int x, int y, double angle){
        //Shear 1
        double tangent = tan(angle/2);
        double new_x = round(x-y*tangent);
        double new_y = y*1.0;

        //Shear 2
        new_y = round(new_x*sin(angle)+new_y);

        //Shear 3
        new_x = round (new_x - new_y * tangent);

        x = (int) new_x;
        y = (int) new_y;

        Coordinates coordinate;
        coordinate.x = x;
        coordinate.y = y;

        return coordinate;
    }
}

