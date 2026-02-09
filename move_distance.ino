/* move desired_distance forward along ground*/
//void move_distance()
//{
//  // reset distance measurement for both right and left wheels
////  rdistance = 0; 
////  ldistance = 0;
//  
//  bool moving = true;
////  while(moving)
////  {
//    Serial.print("distanceRight ="); Serial.print(rdistance);
//    Serial.print("distanceLeft ="); Serial.println(ldistance);
//    if (abs(rdistance) > desired_distance)
//    {
//      front = 0; back = 30; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
//    }
//    else if (abs(rdistance) < desired_distance)
//    {
//      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
//    }
//    else
//    {
//      // reached desired distance, so stop
//      front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0;
//      flag_buzzer = true; // sound the buzzer for short time
//      moving = false; // stop the move_distance() function
//    }
//  //}
//}
