#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;

int main(int argc, char** argv){


Quaterniond q1(0.35,0.2,0.3,0.1), q2(-0.5,0.4,-0.1,0.2);
q1.normalize();
q2.normalize();
Isometry3d T1W(q1);
Isometry3d T2W(q2);

Vector3d t1(0.3,0.1,0.1);
Vector3d t2(-0.1,0.5,0.3);
Vector3d p1(0.5,0,0.2);
T1W.pretranslate(t1);
T2W.pretranslate(t2);

Vector3d p2 = T2W*T1W.inverse()*p1;
cout<<p2 <<endl;
}