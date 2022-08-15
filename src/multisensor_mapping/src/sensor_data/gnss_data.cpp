#include "multisensor_mapping/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <ostream>

double multisensor_mapping::GNSSData::origin_latitude = 0.0;
double multisensor_mapping::GNSSData::origin_longitude = 0.0;
double multisensor_mapping::GNSSData::origin_altitude = 0.0;
bool multisensor_mapping::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian multisensor_mapping::GNSSData::geo_converter;

namespace multisensor_mapping {
void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);

    origin_latitude = latitude;
    origin_longitude = longitude;
    origin_altitude = altitude;

    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }
    // convert geodetic to local cartesian coordinates
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GNSSData::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt) {
    
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }

    // convert local cartesian back to geodetic coordinates
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}

}