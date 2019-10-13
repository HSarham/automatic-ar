#ifndef ARUCOSERDES_H
#define ARUCOSERDES_H

#include <aruco/aruco.h>

class ArucoSerdes
{
public:
    ArucoSerdes();
    static void serialize_marker(aruco::Marker marker, std::ostream& output);
    static void deserialize_marker(std::istream& input, aruco::Marker& marker);
};

#endif // ARUCOSERDES_H
