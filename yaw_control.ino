#include <Arduino.h>
// 变量定义
float yaw_control(float yaw_angle){
// **改进：使用角速度更新累计偏航角**
yaw_rate = yaw_angle - accumulated_yaw;

if (yaw_rate > 180) yaw_rate -= 360;  // 不触发
if (yaw_rate < -180) yaw_rate += 360; // 变成 yaw_rate = +6°（正确）

accumulated_yaw += 0.8* yaw_rate;  // 累计真实旋转角度

// 设置目标角度（仅在 millis() > 15000 时设置一次）
// 
if (millis() > 15000 && i < 1){
  yaw_target = accumulated_yaw + 0;
  i++;
}
if (millis() > 20000 && i < 2){
  yaw_target = accumulated_yaw - 0;
  i++;
}

// 计算误差
if (millis() > 15000){
yaw_error = yaw_target - accumulated_yaw;
}

// if (abs(yaw_error) < 5) {  
//     rdistance = 0;
//     ldistance = 0;
//     yaw_error = 0.3*yaw_error;
    
// }

return yaw_error;

}
