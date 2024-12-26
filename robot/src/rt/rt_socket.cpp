//
// Created by user on 2020/9/26.
//
#include <cstring>
#include "rt/rt_socket.h"
#include <unistd.h>
#include "common/include/Utilities/PeriodicTask.h"
#define DISPLAY_INIT
struct sockaddr_in servaddr, cliaddr;
int servfd,clifd;
bool connected=false;
ReplyMessage replymessage;
Client_Command_Message receive_message;

struct timeval timeout = {5, 0};//用于socket接收超时，设置超时时间5s
//TCPClientMessage tcpclientmsg;
//TCPReplyMessag senddata;

int flag_TCPSend_50Hz;


void openTCP()
{
#ifdef DISPLAY_INIT
    std::cout<<"\t begin to make the thread to TCP-Thread "<<std::endl;
    std::cout<<"\t TCP: creat Socket"<<std::endl;
#endif
    while((servfd=socket(AF_INET,SOCK_STREAM,0))==-1)
    {
        printf("\t OpenTCP : Create Socket error! %s\n",strerror(errno));
        sleep(1);
    }
#ifdef DISPLAY_INIT
    std::cout<<"\t TCP: initialize Socket attribution"<<std::endl;
#endif
    bzero(&servaddr,sizeof(struct sockaddr_in));
    servaddr.sin_family=AF_INET; //ipv4
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port=htons(SERV_PORT);

#ifdef DISPLAY_INIT
    std::cout<<"\t TCP: bind Socket with attribution"<<std::endl;
#endif
    while(bind(servfd, (struct sockaddr *)&servaddr, sizeof(struct sockaddr)) == -1)
    {
        printf("OpenTCP:bind error!%s\n",strerror(errno));
        sleep(1);
    }
#ifdef DISPLAY_INIT
    std::cout<<"\t TCP: Socket listening"<<std::endl;
#endif
    while(listen(servfd,100)==-1)
    {
        printf("OpenTCP:Listen error!%s\n",strerror(errno));
        sleep(1);
    }
#ifdef DISPLAY_INIT
    std::cout<<"\t listening out"<<std::endl;
#endif
}
void TCPSend(){
    flag_TCPSend_50Hz = 1;
}
void* TCPThread(void *arg )
{
    std::cout<<arg<<std::endl;
    PeriodicTaskManager TCPSendManager;
    PeriodicFunction TCPSendTask(&TCPSendManager, 0.1, "TCPSend", &TCPSend);
    int sin_size;
    int ret;
    (void)ret;
    sleep(1);
    replymessage.start = 0xAAAAAAAA;
    replymessage.end = 0xFFFFFFFF;
#ifdef DISPLAY_INIT
    std::cout<<"TCP thread begin running "<<std::endl;
#endif
    openTCP();
#ifdef DISPLAY_INIT
    std::cout<<"TCP thread come to while(1) "<<std::endl;
#endif
    while(1)
    {
        if(!connected)
        {
            sin_size = sizeof(struct sockaddr_in);
#ifdef DISPLAY_INIT
            std::cout<<"\t Server Socket wait to be connected ......"<<std::endl;
#endif

            while((clifd=accept(servfd,(struct sockaddr*) &cliaddr, (socklen_t*) &sin_size))==-1) //
            {
                printf("Accept error:%s\n\a",strerror(errno));
            }
            connected = true;
            //用于网络超时
            //setsockopt(clifd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));
#ifdef DISPLAY_INIT
            std::cout<<"\t Server Socket connected with "<<inet_ntoa(cliaddr.sin_addr) <<std::endl;
#endif
            //	printf("Server get connection from %s\n",inet_ntoa(cliaddr.sin_addr));
//            PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
//                    &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
//            spiTask.start();
        }
        else
        {
            if((ret=recv(clifd,&receive_message,sizeof(Client_Command_Message),0))==-1) //TCP
            {
                printf("Read Error:%s\n",strerror(errno));
                //用于网络超时
                //receive_message.mode = 15;
                //receive_message.velocity_x = 0.0;
                //receive_message.velocity_y = 0.0;
                //receive_message.omega_z = 0.0;
                connected=false;
                continue;
            }
            if(ret ==0)//TCP
            {
                printf("Read error,Connection off!\n");
                connected=false;
                continue;
            }
//            std::cout << "Recv:" << receive_message.mode << std::endl;

            /*if((ret==sizeof(Client_Command_Message)) && (receive_message.start == 0xAAAAAAAA)&&(receive_message.end == 0x0FFFFFFF))
            {

                std::cout<<"tcp command: mode"<<receive_message.mode<<"\t vx"<<receive_message.velocity_x<<"\t"<<std::endl;
//                printf("tcp receive ok\n");
                replymessage.velocity_x=receive_message.velocity_x;
                replymessage.velocity_y=receive_message.velocity_y;
                replymessage.omega_z=receive_message.omega_z;

                if( (ret = send(clifd, &replymessage, sizeof(replymessage), 0) < 0)){
                    printf("send error\n");
                }
            }
            else
            {
               printf("Data error!!!!!\n");
            }*/
        }
    }
    close(servfd);
    return EXIT_SUCCESS;
}
//TCP发送线程，每50ms发送一次机器人数据
void* TCPSendThread(void *arg ){
    (void)arg;
    int ret = 0;
    while(1){
        if(connected == true){
            //std::cout << "Send Thread" << std::endl;
            if( (ret = send(clifd, &replymessage, sizeof(replymessage), 0) < 0)){
                printf("send error\n");
            }
            usleep(20000);
        }
        else{
            sleep(1);
        }
    }
}
void init_TCPThread()
{
    int ret;
    pthread_t id, sid;
    (void)sid;
    pthread_attr_t threadattr;
#ifdef DISPLAY_INIT
    std::cout<<"\t init thread attribution"<<std::endl;
#endif
    pthread_attr_init(&threadattr);
    pthread_attr_setdetachstate(&threadattr,PTHREAD_CREATE_DETACHED);
    //pthread_create()
#ifdef DISPLAY_INIT
    std::cout<<"\t creat thread"<<std::endl;
#endif
    //ret = pthread_create(&id,&threadattr,&TCPThread,NULL);
    ret=pthread_create(&id,NULL,&TCPThread,NULL);
    std::cout<<"tcp thead id: "<<ret <<std::endl;
#ifdef DISPLAY_INIT
    if(ret==0)
        std::cout<<"\t pthread created OK"<<std::endl;
	else
        std::cout<<"\t pthread create error"<<std::endl;
#endif
	//TCP发送线程
    ret=pthread_create(&sid,NULL,&TCPSendThread,NULL);
    std::cout<<"tcp send thead id: "<<ret <<std::endl;
#ifdef DISPLAY_INIT
    if(ret==0)
        std::cout<<"\t pthread created OK"<<std::endl;
    else
        std::cout<<"\t pthread create error"<<std::endl;
#endif
}