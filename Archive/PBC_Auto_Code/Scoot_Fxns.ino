float distance_to(float lat1, float lat2, float lon1, float lon2) {

  int r = 6371; //radius of the world in km
  float dlat = radians(lat2 - lat1);
  float dlon = radians(lon2 - lon1);
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2))
            * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = (float) r * c;
  return d;
}
//===========================================================================================Calculate Distance=======
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===========================================================================================Calculate Course=========
float course_to(float lat1, float lat2, float lon1, float lon2) {
  float lat_dif;
  float lon_dif;
  float course_;

  lat_dif = lat1 - lat2;
  if (lat_dif < 0) {
    lat_dif *= (-1.0);
  }
  lon_dif = lon1 - lon2;
  if (lon_dif < 0) {
    lon_dif *= (-1.0);
  }

  lat_dif *= 60.0 * 1852.0;

  lon_dif *= 60.0;
  lon_dif *= cos(lat2 * PI / 180);
  lon_dif *= 1852.0;

  course_ = atan(lon_dif / lat_dif) * 180 / PI;

  if ((lat2 < lat1) && (lon2 > lon1)) {
    course_ = 180.0 - course_;
  }
  else if ((lat2 < lat1) && (lon2 < lon1)) {
    course_ = course_ + 180.0;
  }
  else if ((lat2 > lat1) && (lon2 < lon1)) {
    course_ = 360.0 - course_;
  }

  return course_;
}
