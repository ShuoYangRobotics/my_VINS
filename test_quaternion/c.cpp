#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "q.h"
using namespace Eigen;
using namespace std;
int main()
{
	srand(0);
	int s = 100;
	while (s--) {
		Vector3d p(rand() % 10 - 5, rand() % 10  - 5, rand() % 10  - 5);
		Vector4d c(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5);
		Vector4d c2(rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5, rand() % 10 - 5);

		Quaterniond q1(c);
		q1.normalize();
		Quaterniond q2(c2);
		q2.normalize();


		QuaternionAlias<double> q3(c);
		QuaternionAlias<double> q4(c2);
		q3.normalize();
		q4.normalize();
		
		Vector3d p1 = q1 * (q2 * p);
		Vector3d p2 = q1.matrix() * q2.matrix() * p;

		cout << q1.toRotationMatrix() << endl;
		cout << q1.matrix() << endl;

		cout << p1.transpose() << endl;
		cout << p2.transpose() << endl;
	}
	return 0;
}
