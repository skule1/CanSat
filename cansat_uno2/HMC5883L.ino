void hmc5883l()
{  float heading;
      mag.getHeading(&mx, &my, &mz);

// To calculate heading in degrees. 0 degree indicates North
     heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;

    dataFile.print(mx); dataFile.print("\t");
    dataFile.print(my); dataFile.print("\t");
    dataFile.print(mz); dataFile.print("\t");
    dataFile.print(heading * 180/M_PI);dataFile.print("\t");
    
    RF_port.print(mx); RF_port.print("\t");
    RF_port.print(my); RF_port.print("\t");
    RF_port.print(mz); RF_port.print("\t");    
    RF_port.print(heading * 180/M_PI); RF_port.print("\t");

    

}

