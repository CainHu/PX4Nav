#include "gvar.h"

// WGS-84 model
const double GM = 3.986004415e14;
const double Re = 6.378136998405e6;
const double wie = 7.2921151467e-5;

const double ff = 1. / 298.257223563;
const double ee = sqrt(2. * ff - ff * ff);
const double e2 = ee * ee;
const double Rp = (1. - ff) * Re;

// gravity, ug
const double ge = 9.780325333434361;
const double gp = 9.832184935381024;
const double g0 = ge;
const double ug = g0 * 1e-6;

// angle unit
const double arcdeg = M_PI / 180.;
const double arcmin = arcdeg / 60.;
const double arcsec = arcmin / 60.;

// hour, deg/hour, deg / sqrt(hour)
const double hur = 3600.;
const double dph = arcdeg / hur;
const double dpsh = arcdeg / sqrt(hur);

// ug/sqrt(Hz)
const double ugpsHz = ug / sqrt(1.);


matrix::Vector3d geo2earth(const matrix::Vector3d &pos) {
    const double cosL = cos(pos(0)), sinL = sin(pos(0));
    const double coslam = cos(pos(1)), sinlam = sin(pos(1));
    const double tmp = 1. - e2 * sinL * sinL;
    const double RN = Re / sqrt(tmp);
    return {(RN + pos(2)) * cosL * coslam, (RN + pos(2)) * cosL * sinlam, (RN * (1. - e2) + pos(2)) * sinL};
}

matrix::Vector3d earth2nav(const matrix::Vector3d &earth, const matrix::Vector3d &geo) {
    matrix::Matrix3d Rne;
    const double cosL = cos(geo(0)), sinL = sin(geo(0));
    const double coslam = cos(geo(1)), sinlam = sin(geo(1));

    Rne(0, 0) = -sinL * coslam;
    Rne(0, 1) = -sinL * sinlam;
    Rne(0, 2) = cosL;
    Rne(1, 0) = -sinlam;
    Rne(1, 1) = coslam;
    Rne(1, 2) = 0.;
    Rne(2, 0) = -cosL * coslam;
    Rne(2, 1) = -cosL * sinlam;
    Rne(2, 2) = -sinL;

    return Rne * earth;       
}

matrix::Vector3d diff_geo(const matrix::Vector3d &geo2, const matrix::Vector3d &geo1) {
    matrix::Matrix3d Rne;
    const double cosL = cos(geo2(0)), sinL = sin(geo2(0));
    const double coslam = cos(geo2(1)), sinlam = sin(geo2(1));

    Rne(0, 0) = -sinL * coslam;
    Rne(0, 1) = -sinL * sinlam;
    Rne(0, 2) = cosL;
    Rne(1, 0) = -sinlam;
    Rne(1, 1) = coslam;
    Rne(1, 2) = 0.;
    Rne(2, 0) = -cosL * coslam;
    Rne(2, 1) = -cosL * sinlam;
    Rne(2, 2) = -sinL;

    matrix::Vector3d earth2 = geo2earth(geo2);
    matrix::Vector3d earth1 = geo2earth(geo1);

    return Rne * (earth2 - earth1);    
}



