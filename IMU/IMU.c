#include "IMU.h"
#include "string.h"


 
//uint16_t Crcnum = 0;
 
void IMU_Receive(IMU_Typedef* Dst,const uint8_t* Data)
{
	if(Data[0] == 0x55)
	{
		for(uint16_t i = 1; i < 30 ; i+=11)
		{
			switch(Data[i])
			{
#if Acceleration_EN == 1 //加速度
                case kItemAcceleration:
                    Dst->Acceleration.X = ((int16_t)(Data[i+2]<<8)|Data[i+1])/32768.0f*16*9.8;
                    Dst->Acceleration.Y = ((int16_t)(Data[i+4]<<8)|Data[i+3])/32768.0f*16*9.8;
					Dst->Acceleration.Z = ((int16_t)(Data[i+6]<<8)|Data[i+5])/32768.0f*16*9.8;
                break;
#endif
#if AngularVelocity_EN == 1//角速度
                case kItemAngularVelocity:					
					Dst->AngularVelocity.X = ((int16_t)(Data[i+2]<<8)|Data[i+1])/32768.0f*2000;
					Dst->AngularVelocity.Y = ((int16_t)(Data[i+4]<<8)|Data[i+3])/32768.0f*2000;
					Dst->AngularVelocity.Z = ((int16_t)(Data[i+6]<<8)|Data[i+5])/32768.0f*2000;
				break;
#endif

#if EulerAngle_EN == 1//欧拉角
				case kItemEulerAngler:
					Dst->EulerAngler.Roll  = ((int16_t)(Data[i+2]<<8)|Data[i+1])/32768.0f*180;
					Dst->EulerAngler.Pitch = ((int16_t)(Data[i+4]<<8)|Data[i+3])/32768.0f*180;
					Dst->EulerAngler.Yaw   = ((int16_t)(Data[i+6]<<8)|Data[i+5])/32768.0f*180;
										
					float diff = Dst->EulerAngler.Yaw - Dst->EulerAngler.LsatAngle;
					if(diff > 100) 
						Dst->EulerAngler.r--;
					else if(diff < -100) 
						Dst->EulerAngler.r++;
								
					Dst->EulerAngler.ContinuousYaw = Dst->EulerAngler.r * 360.0f
																		+ Dst->EulerAngler.Yaw;		
					Dst->EulerAngler.LsatAngle = Dst->EulerAngler.Yaw;
				break;
#endif
			}
		}		
	}
}

