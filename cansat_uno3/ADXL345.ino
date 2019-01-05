void adxl345()
{


/*




  
    accel.getAcceleration(&ax, &ay, &az);

    dataFile.print(ax); dataFile.print("\t");
    dataFile.print(ay); dataFile.print("\t");
    dataFile.print(az); dataFile.print("\t");
    
    RF_port.print(ax); RF_port.print("\t");
    RF_port.print(ay); RF_port.print("\t");
    RF_port.print(az); RF_port.print("\t");


*/
const float alpha = 0.5;
float pitch, roll;
int16_t Xg, Yg, Zg;
float fXg = 0;
float fYg = 0;
float fZg = 0;
 
    accel.getAcceleration(&Xg, &Yg, &Zg);

//    acc.read(&Xg, &Yg, &Zg);
 
    //Low Pass Filter
    fXg = Xg * alpha + (fXg * (1.0 - alpha));
    fYg = Yg * alpha + (fYg * (1.0 - alpha));
    fZg = Zg * alpha + (fZg * (1.0 - alpha));
 
    //Roll & Pitch Equations
    roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
#ifdef SD_Brikke
   dataFile.print(Xg); dataFile.print("\t");
    dataFile.print(Yg); dataFile.print("\t");
    dataFile.print(Zg); dataFile.print("\t");
#endif    
    RF_port.print(Xg); RF_port.print("\t");
    RF_port.print(Yg); RF_port.print("\t");
    RF_port.print(Zg); RF_port.print("\t");



#ifdef SD_Brikke
    
//  dataFile.print("pitch  ");
    dataFile.print(pitch);dataFile.print("\t");
//    dataFile.print("roll ");  
    dataFile.print(roll);dataFile.print("\t");
#endif

//  RF_port.print("pitch  ");
    RF_port.print(pitch);RF_port.print("\t");
//  RF_port.print("roll ");
    RF_port.print(roll);RF_port.print("\t");
}
