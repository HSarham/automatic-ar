#include "aruco_serdes.h"
#include <iostream>
using namespace std;
ArucoSerdes::ArucoSerdes()
{

}

void ArucoSerdes::serialize_marker(aruco::Marker marker, ostream& output){
    output.write((char*)&marker.id,sizeof(marker.id));
    for(int c=0;c<4;c++){
        output.write((char*)&marker[c].x,sizeof(marker[c].x));
        output.write((char*)&marker[c].y,sizeof(marker[c].y));
    }
}

void ArucoSerdes::deserialize_marker(istream& input, aruco::Marker& marker){
    input.read((char*)&marker.id,sizeof(marker.id));
    marker.resize(4);
    for(int c=0;c<4;c++){
        input.read((char*)&marker[c].x,sizeof(marker[c].x));
        input.read((char*)&marker[c].y,sizeof(marker[c].y));
    }
}
