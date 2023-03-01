//
// Created by Cain on 2023/3/1.
//

#ifndef GEO_MAG_DECLINATION_H
#define GEO_MAG_DECLINATION_H

// Return magnetic declination in degrees or radians
float get_mag_declination_degrees(float lat, float lon);
float get_mag_declination_radians(float lat, float lon);

// Return magnetic field inclination in degrees or radians
float get_mag_inclination_degrees(float lat, float lon);
float get_mag_inclination_radians(float lat, float lon);

// return magnetic field strength in Gauss or Tesla
float get_mag_strength_gauss(float lat, float lon);
float get_mag_strength_tesla(float lat, float lon);


#endif //GEO_MAG_DECLINATION_H
