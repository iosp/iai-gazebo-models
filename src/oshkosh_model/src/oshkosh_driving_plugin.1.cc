    private: void break(physics::JointPtr joint,int break)
    {
      gazebo::math::Angle currentPosition = joint->GetAngle(0);
        // std::cout << " getting angle"<< std::endl;
        if(break)
        {
          joint->SetHighStop(0, currentPosition);
          joint->SetLowStop(0, currentPosition); 
        }
          else
          {
            joint->SetHighStop(0, gazebo::math::Angle(10000000000000000));
            joint->SetLowStop(0, gazebo::math::Angle(-10000000000000000));
          }
         
        // std::cout << "efforting"<< std::endl;
        // this->jointController->SetJointPosition(steer_joint, Angle*0.61);

    }

    private: void On_Break_command(const std_msgs::Float64ConstPtr &msg)
    {
      int Break;
    //  Break_mutex.lock();
          // Recieving referance velocity
          if(msg->data >= 1)       Break =  1;
          else                     Break = 0;
          
        break(this->left_wheel_1 , Break);
        break(this->left_wheel_2 , Break);
        break(this->left_wheel_3 , Break);
        break(this->left_wheel_4 , Break);
        break(this->right_wheel_1, Break);
        break(this->right_wheel_2, Break);
        break(this->right_wheel_3, Break);
        break(this->right_wheel_4, Break);

        // Reseting timer every time LLC publishes message
// #if GAZEBO_MAJOR_VERSION >= 5
//            Linear_command_timer.Reset();
// #endif
//            Linear_command_timer.Start();

//       Linear_command_mutex.unlock();
    }
