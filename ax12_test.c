#include "ax12_openCM.h"
#define CONVERT 0.29296875

Dynamixel Dxl;


void print_useage()
{
    printf("====== Dynamixel servo test demo ======\n");
    printf("cmds:\n");
    printf("p\tget current postion(joint model)\n");
    printf("P\tgo to postion 0(joint model)\n");
    printf("s\tloop: go to postion 0 sleep 1s go to postion 300 sleep 1s(joint model)\n");
}

void demo_model()
{
    while(1)
    {
        //Turn dynamixel ID 1 to position 0
        Dxl.writeWord(1, 30, 0); //Compatible with all dynamixel serise
        // Wait for 1 second (1000 milliseconds)
        sleep(1);
        //Turn dynamixel ID 1 to position 300
        Dxl.writeWord(1, 30, 300);
        // Wait for 1 second (1000 milliseconds)
        sleep(1);
    }
}

int main()
{

    int ret;
    ret = Dxl.begin(1000000);
    if (ret == -1)
    {
        return 0;
    }
    int model = Dxl.getModelNumber(1);
    if(model == 12)
        printf("AX-12A\n");
    Dxl.jointMode(1);
    char cmd;
    double degree;
    while (1)
    {
        printf("please input command:");
        scanf("%c\n", &cmd);
        switch (cmd)
        {
        case 'p':
            ret = Dxl.getPosition(1);
            printf("ret = %d \n", ret);
            degree = ret * CONVERT;
            printf("current is %f degree\n", degree);
            break;
        case 'P':
            Dxl.writeWord(1, GOAL_POSITION, 0);
            break;
        case 's':
            demo_model();
            break;
        default:
            print_useage();
            break;
        }
    }

    return 0;
    //Dxl.writeWord(1, 30, ((int)(p1)) * 256 + (int)p2);
}
