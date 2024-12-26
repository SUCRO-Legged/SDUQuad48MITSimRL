#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <lcm/lcm-cpp.hpp>
#include <time.h>
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "vectornav_lcmt.hpp"


char ptr_state[70] = {'\0'};
char ptr_command[70] = {'\0'};
char ptr_data[70] = {'\0'};
char ptr_imu[70] = {'\0'};


using namespace std;    
class Leg_Command_Handler 
{
public:
	~Leg_Command_Handler() {}
     
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    	   const std::string& chan, 
                           const leg_control_command_lcmt* msg)
        { 
		static int i = 0;              
		static ofstream log_leg_com(ptr_command);//"/home/user/log/log_leg_control_command.csv");
		if(i > 10)
		{	
			i = 0;
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->tau_ff[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->f_ff[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->q_des[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->qd_des[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->p_des[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->v_des[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->kp_cartesian[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->kd_cartesian[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->kp_joint[i] << ",";
			}
			log_leg_com<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_com << msg->kd_joint[i] << ",";
			}
			log_leg_com<<endl;  
		
		} 
		i++;     
	}
};
class Leg_data_Handler
{
public:
	~Leg_data_Handler() {}
     
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    	   const std::string& chan, 
                           const leg_control_data_lcmt* msg)
        { 
		static int i = 0;            
		static ofstream log_leg_data(ptr_data);//"/home/user/log/log_leg_control_data.csv");
		if(i > 10)
		{
			i = 0;
		 	for(int i = 0; i < 12; i++)
			{
				log_leg_data << msg->q[i] << ",";
			}
			log_leg_data<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_data << msg->qd[i] << ",";
			}
			log_leg_data<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_data << msg->p[i] << ",";
			}
			log_leg_data<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_data << msg->v[i] << ",";
			}
			log_leg_data<<",";
			for(int i = 0; i < 12; i++)
			{
				log_leg_data << msg->tau_est[i] << ",";
			}
			log_leg_data<<endl; 
                     
		}
		i++;
	}
};
class State_Estimator_Handler 
{
public:
	~State_Estimator_Handler() {}
     
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    	   const std::string& chan, 
                           const state_estimator_lcmt* msg)
        {
		static int i = 0;              
		static ofstream log_state(ptr_state);//"/home/user/log/log_state_estimator.csv");
		if(i > 10)
		{
			i = 0;
			log_state<< msg->p[0]<<",";
			log_state<< msg->p[1]<<",";
			log_state<< msg->p[2]<<",";
			log_state<<",";
			log_state<<msg->vWorld[0]<<",";
			log_state<<msg->vWorld[1]<<",";
			log_state<<msg->vWorld[2]<<",";
			log_state<<",";
			log_state<<msg->vBody[0]<<",";
			log_state<<msg->vBody[1]<<",";
			log_state<<msg->vBody[2]<<",";
			log_state<<",";
			log_state<<msg->rpy[0]<<",";
			log_state<<msg->rpy[1]<<",";
			log_state<<msg->rpy[2]<<",";
			log_state<<",";
			log_state<<msg->omegaBody[0]<<",";
			log_state<<msg->omegaBody[1]<<",";
			log_state<<msg->omegaBody[2]<<",";
			log_state<<",";
			log_state<<msg->omegaWorld[0]<<",";
			log_state<<msg->omegaWorld[1]<<",";
			log_state<<msg->omegaWorld[2]<<",";
			log_state<<",";
			log_state<<msg->quat[0]<<",";
			log_state<<msg->quat[1]<<",";
			log_state<<msg->quat[2]<<",";
			log_state<<msg->quat[3]<<",";
			log_state<<",";
			log_state<<msg->aBody[0]<<",";
			log_state<<msg->aBody[1]<<",";
			log_state<<msg->aBody[2]<<",";
                        log_state<<",";
			log_state<<msg->aWorld[0]<<",";
			log_state<<msg->aWorld[1]<<",";
			log_state<<msg->aWorld[2]<<",";
                        log_state<<",";
			log_state<<msg->vRemoter[0]<<",";
			log_state<<msg->vRemoter[1]<<",";
			log_state<<msg->vRemoter[2]<<",";
			log_state<<endl;
			printf("rRemoterVelociy: %.2f\t%.2f\t%.2f\t\t %.2f\t%.2f\t%.2f\n",
					msg->vRemoter[0],msg->vRemoter[1],msg->vRemoter[2],
					msg->vBody[0],msg->vBody[1],msg->vBody[2]);

		}
		i++;           
	}
};    
class IMU_Handler 
{
public:
	~IMU_Handler() {}
     
	void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    	   const std::string& chan, 
                           const vectornav_lcmt* msg)
        {
		static int i = 0;              
		static ofstream log_imu(ptr_imu);//"/home/user/log/log_imu.csv");
		if(i > 10)
		{
			i = 0;
			log_imu<<msg->q[0]<<","; log_imu<<msg->q[1]<<","; log_imu<<msg->q[2]<<","; log_imu<<msg->q[3]<<","; log_imu<<",";
			log_imu<<msg->w[0]<<","; log_imu<<msg->w[1]<<","; log_imu<<msg->w[2]<<","; log_imu<<",";
			log_imu<<msg->a[0]<<","; log_imu<<msg->a[1]<<","; log_imu<<msg->a[2]<<","; log_imu<<endl;
		}
		i++;           
	}
};    
int main(int argc, char** argv)
{
	time_t tim;
	struct tm *tp;
	char ptr_dir[70] = {'\0'};
	time(&tim);
	tp = gmtime(&tim);
	
	sprintf(ptr_dir,     "/home/user/log/%d-%d-%d-%d-%d-%d/", 1900+tp->tm_year, 1+tp->tm_mon, tp->tm_mday, 8+tp->tm_hour, tp->tm_min, tp->tm_sec);
	mkdir(ptr_dir, S_IRWXU|S_IRWXG|S_IRWXO);	

	sprintf(ptr_command, "/home/user/log/%d-%d-%d-%d-%d-%d/log_leg_control_command.csv", 1900+tp->tm_year, 1+tp->tm_mon, tp->tm_mday, 8+tp->tm_hour, tp->tm_min, tp->tm_sec);
	sprintf(ptr_data,    "/home/user/log/%d-%d-%d-%d-%d-%d/log_leg_control_data.csv",    1900+tp->tm_year, 1+tp->tm_mon, tp->tm_mday, 8+tp->tm_hour, tp->tm_min, tp->tm_sec);
	sprintf(ptr_state,   "/home/user/log/%d-%d-%d-%d-%d-%d/log_leg_control_state.csv",   1900+tp->tm_year, 1+tp->tm_mon, tp->tm_mday, 8+tp->tm_hour, tp->tm_min, tp->tm_sec);
	sprintf(ptr_imu,   "/home/user/log/%d-%d-%d-%d-%d-%d/log_imu.csv",   1900+tp->tm_year, 1+tp->tm_mon, tp->tm_mday, 8+tp->tm_hour, tp->tm_min, tp->tm_sec);
	
	lcm::LCM lcm;
	if(!lcm.good())
    		return 1;

	Leg_Command_Handler 	leg_command_obj;
	Leg_data_Handler    	leg_data_obj;
	State_Estimator_Handler state_obj;
	IMU_Handler             imu_obj;
		
	cout << "-----------------starting record data--------------------" << endl;	
	lcm.subscribe("leg_control_command", &Leg_Command_Handler::handleMessage,     &leg_command_obj);
	lcm.subscribe("leg_control_data",    &Leg_data_Handler::handleMessage,        &leg_data_obj);
	lcm.subscribe("state_estimator",     &State_Estimator_Handler::handleMessage, &state_obj);
	lcm.subscribe("hw_vectornav",        &IMU_Handler::handleMessage, &imu_obj);

	while(0 == lcm.handle());

	return 0;
}
