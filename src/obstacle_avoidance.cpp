const float tol = 1;
//propGain and derGain will be changed as needed
const float propGain = 1;
const float derGain = 1;
vector<float> oldDist = {tol, tol, tol, tol};
vector<float> *oldDistPtr = &oldDist;
vector<float> getSonars();
int avoid() {
   vector<float> dist; //order: north, east. south, west
   vector<float> prop; //proportion (how close is the object we need to avoid)
   prop = getSonars();
   //int sum = 0;
   float h = 0.0;
   for(float i : prop)
   {
       if(i<tol)
           dist.push_back(i);
       else
           dist.push_back(tol);
   }
   float netXDist = dist[1] - dist[3];
   float netYDist = dist[0] - dist[2];
   vector<float> der;
   for (int i = 0; i < dist.size(); i++) {
       der.push_back(dist[i] - oldDist[i]);
   }
   float netXDer = der[1] - der[3];
   float netYDer = der[0] - der[2];
   vector<float> velocities = {propGain * netXDist + derGain * netXDer, propGain * netYDist + derGain * netYDer};
   //forward,right,down,left
   //pre-transformation
   if (velocities[0] == 0) {
       set_destination(current_pose_g.pose.pose.position.x + waypoint.x,
                       current_pose_g.pose.pose.position.y + velocities[1],
                       current_pose_g.pose.pose.position.z + h, 0);
   } else if (velocities[1] == 0) {
       set_destination(current_pose_g.pose.pose.position.x + velocities[1],
                       current_pose_g.pose.pose.position.y + waypoint.y,
                       current_pose_g.pose.pose.position.z + h, 0);
   } else {
       set_destination(current_pose_g.pose.pose.position.x + velocities[0],
                       current_pose_g.pose.pose.position.y + velocities[1],
                       current_pose_g.pose.pose.position.z + h, 0);
   }
   *oldDistPtr = dist;
   return 1;
}
