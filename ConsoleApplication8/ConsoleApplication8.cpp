/* PLEASE READ

This Lab 1 Assignment is completed by Chenkai(Tim) Ling and Tianyu Deng

Both student contribute to the code.

Chenkai(Tim) Ling contribution:

1. Build user interface for input and command
2. Implement and create forward kinematics method with Tianyu deng
3. Implement and create inverse kinematics method with Tianyu deng
4. Write functions for matrix and parameter calculation.

Tianyu Deng contribution:

1. Design and prove inverse kinematic algorithm with Tim
2. Debug and improve the performance of code
3. Calculate forward kinematic algorithm
4. Provide github account and repository

*/











// ConsoleApplication8.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <iostream>
#include <iomanip>

using namespace std;

#define PI 3.14159265
#define LD 14.1421356

int choice = 0;
char cfk = '0';
char cik = '0';

/* forward K calculation*/

int link = 0;
int i = 1;
char loopc = '0';
//User input (x,y,z)
double x = 0;
double y = 0;
double z = 0;

double ai = 0;  //Link Length
double aai = 0; //Link Twist
double di = 0;  //Link Offset
double ddi = 0; //Joint Angle

				// Matrix element
double r11 = 0;
double r12 = 0;
double r13 = 0;
double r21 = 0;
double r22 = 0;
double r23 = 0;
double r31 = 0;
double r32 = 0;
double r33 = 0;

double Z = 0;

double o = 1;

double t1 = 0;
double t2 = 0;
double t3 = 0;

//Calculated/Result (x,y,z)
double xx = 0;
double yy = 0;
double zz = 0;



/* Inverse K calculation*/

//Origin/User input (x,y,z)
double inx = 0;
double iny = 0;
double inz = 0;
//End/Destination (x,y,z)
double enx = 0;
double eny = 0;
double enz = 0;
//Present (x,y,z)
double prx = 0;
double pry = 0;
double prz = 0;



//Deltax for Jacobian
double deltax = 0;
//Total distance
double tdistance = 0;
//Max reach of robotic arm
double maxdistance = 0;
//Total distance for 2,3,4 link to reach
double redistance = 0;

// ai--link length, aai--link twist, di--link offset, ddi--joint angle
double a1 = 0;
double aa1 = 0;
double d1 = 0;
double dd1 = 0;

double a2 = 0;
double aa2 = 0;
double d2 = 0;
double dd2 = 0;

double a3 = 0;
double aa3 = 0;
double d3 = 0;
double dd3 = 0;

double a4 = 0;
double aa4 = 0;
double d4 = 0;
double dd4 = 0;

//Joint 2 position
double lp1x = 0;
double lp1y = 0;
double lp1z = 0;
//Matrix element temperary storage and update
double n11 = 0;
double n12 = 0;
double n13 = 0;
double n21 = 0;
double n22 = 0;
double n23 = 0;
double n31 = 0;
double n32 = 0;
double n33 = 0;
double n1t = 0;
double n2t = 0;
double n3t = 0;

double nn11 = 0;
double nn12 = 0;
double nn13 = 0;
double nn21 = 0;
double nn22 = 0;
double nn23 = 0;
double nn31 = 0;
double nn32 = 0;
double nn33 = 0;
double nn1t = 0;
double nn2t = 0;
double nn3t = 0;

/* This is just for trajetory calculation*/

double nd1 = 0; //new joint angles
double nd2 = 0;
double nd3 = 0;
double nd4 = 0;

double trax = 0;//increment in movement
double tray = 0;
double traz = 0;

double tdx = 0;
double tdy = 0;
double tdz = 0;

void FK(void); // For FK calculation and user interface


void vecal(double wc, double xc, double yc, double zc, double sx, double sy, double sz);//New (x,y,z) position calculation inputing ai,aai,di,ddi and original (x,y,z)   D-H to position.

																						/*4x4 to 4x4 matrix calculation*/
void MatrixTMatrix(double rr11, double rr12, double rr13, double rt1, double rr21, double rr22, double rr23, double rt2, double rr31, double rr32, double rr33, double rt3, double nr11, double nr12, double nr13, double nt1, double nr21, double nr22, double nr23, double nt2, double nr31, double nr32, double nr33, double nt3);
/*4x4 to (x,y,z,1) matrix calculation*/
void MatrixTVec(double rr11, double rr12, double rr13, double rt1, double rr21, double rr22, double rr23, double rt2, double rr31, double rr32, double rr33, double rt3, double npx, double npy, double npz);

void IK(void);// IK user interface

void vecalw(double wc, double xc, double yc, double zc, double sx, double sy, double sz);//New (x,y,z) position calculation inputing ai,aai,di,ddi and original (x,y,z)   D-H to position without display on screen.


int main()
{
	while (1) {
		//Get user input
		std::cout.precision(5);
		cout << "Welcome to ICT 4 link robotic arm FK and IK calculation " << endl;
		cout << "Pick FK or IK option " << endl;
		cout << "Enter 1 for FK, Enter 2 for IK" << endl;
		cin >> choice;
		if (choice == 1)
		{

			FK();
		}
		else if (choice == 2)
		{
			IK();
		}
		else if ((choice != 1) && (choice != 2))
		{
			cout << "Please type in only 1 or 2" << endl;
		}
	}
	return 0;
}


/* In this section, it starts with getting original position from user. Then it asks user to
input four D-H parameters. Base on D-H parameters, the code generates a D-H matrix. This D-H matrix
is displayed on the screen. Then the code do matrix multiplication to update T matrix every time
it moves to next link. The updated/calculated T matrix is referenced to origin point. Therefore,
user can have a clear picture at where the joint is relate to world frame.

Detail procedure is as following:

Input origin point
Input 4 D-H parameter



*/











void FK(void)//forward calculation
{

	cout << "You are doing FK Calculation" << endl << endl;
	cout << "Please enter number of links of the machine" << endl;
	cout << "Number of links =  ";
	cin >> link;

	cout << "Please enter your original (x,y,z)" << endl;
	cout << "x =  ";
	cin >> x;
	cout << "y =  ";
	cin >> y;
	cout << "z =  ";
	cin >> z;



	while (link >= 1)
	{

		cout << "Please enter D-H parameters for link  " << i << "  " << endl;
		cout << "Link length (cm) =  ";
		cin >> ai;
		cout << "Link twist (degree) =  ";
		cin >> aai;
		cout << "Link offset (cm) =  ";
		cin >> di;
		cout << "Joint angle (degree) =  ";
		cin >> ddi;
		link--;
		cout << "Your T" << i - 1 << " to " << i << " looks like" << endl;
		nn11 = r11;
		nn12 = r12;
		nn13 = r13;
		nn21 = r21;
		nn22 = r22;
		nn23 = r23;
		nn31 = r31;
		nn32 = r32;
		nn33 = r33;
		nn1t = t1;
		nn2t = t2;
		nn3t = t3;
		if (i > 2)
		{
			nn11 = n11;
			nn12 = n12;
			nn13 = n13;
			nn21 = n21;
			nn22 = n22;
			nn23 = n23;
			nn31 = n31;
			nn32 = n32;
			nn33 = n33;
			nn1t = n1t;
			nn2t = n2t;
			nn3t = n3t;
		}
		vecal(ddi, aai, ai, di, 0, 0, 0);
		//Store old matrix elements.

		if (i == 1)
		{
			MatrixTVec(r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3, 0, 0, 0);
		}
		if (i != 1)
		{
			MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

			MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);
		}
		if (link != 0)
		{
			cout << "Your new Joint (x,y,z) point is: " << endl;
		}
		if (link == 0)
		{
			cout << "End effector position is: " << endl;
		}
		cout << "(" << xx + x << ", " << yy + y << ", " << zz + z << ")" << endl;
		cout << endl;
		cout << endl;
		cout << endl;
		cout << "Press e to continue" << endl;
		cin >> cfk;
		while (1)
		{
			if (cfk == 'e' || cfk == 'E')
			{
				break;
			}
		}
		i++;
	}
}
void IK(void)
{
	choice = 0;
	cout << "You are doing IK Calculation" << endl;
	cout << "This calculation only applies for 4-link ICT robotic arm" << endl << endl;
	cout << "Please enter base position (x,y,z)" << endl;
	cout << "x = ";
	cin >> inx;
	cout << "y = ";
	cin >> iny;
	cout << "z = ";
	cin >> inz;

	cout << "Enter 1 for Joint position calculation.  2 for trajectory calculation" << endl;
	cin >> choice;

	if (choice == 1)
	{
		cout << "Please enter end-effector position (x,y,z)" << endl;
		cout << "x = ";
		cin >> enx;
		cout << "y = ";
		cin >> eny;
		cout << "z = ";
		cin >> enz;

		enx = enx - inx;
		eny = eny - iny;
		enz = enz - inz;
		dd1 = atan(eny / enx) * 180 / PI;
		if ((eny < 0 && enx<0) || (eny>0 && enx<0))
		{
			dd1 = 180 + dd1;
		}

		d1 = 20 * sin(45 * PI / 180);
		a1 = 20 * sin(45 * PI / 180);
		aa1 = -90;

		vecal(dd1, aa1, a1, d1, 0, 0, 0);

		lp1x = xx;
		lp1y = yy;
		lp1z = zz;

		maxdistance = 20 * cos(45 * PI / 180) + 40 + 20 + 20;
		tdistance = sqrt((enx - 0)*(enx - 0) + (eny - 0)*(eny - 0) + (enz - 0)*(enz - 0));

		if (tdistance > maxdistance)
		{
			cout << "Error! too far for robotic arm to reach, max distance is " << maxdistance << endl;
			return;

		}


		redistance = sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y) + (enz - lp1z)*(enz - lp1z));



		if ((enx*enx + eny*eny) > (lp1x*lp1x + lp1y*lp1y))
		{
			if (redistance < 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
				dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;
			}
			else if (redistance == 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = 180;
				dd4 = 180;
			}
			else if (redistance > 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = acos((redistance - 40) / 2 / 20) * 180 / PI;
				dd4 = -2 * acos((redistance - 40) / 2 / 20) * 180 / PI;
			}

		}
		if ((enx*enx + eny*eny) < (lp1x*lp1x + lp1y*lp1y))
		{
			if (enz < 14.1421)
			{
				dd2 = 180 - (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180);

			}
			if (enz > 14.1421)
			{
				dd2 = (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180) - 90;

			}
			dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
			dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;

		}
		cout << endl;
		cout << "Four link angles are: " << endl;
		cout << dd1 << "," << dd2 << "," << dd3 << "," << dd4 << "," << endl;
		//fixed link 2 variable
		d2 = 0;
		a2 = 40;
		aa2 = 0;
		//fixed link 3 variable
		d3 = 0;
		a3 = 20;
		aa3 = 0;
		//fixed link 4 variable
		d4 = 0;
		a4 = 20;
		aa4 = 0;


		cout << "Your T0-1 looks like" << endl;
		vecal(dd1, aa1, a1, d1, 0, 0, 0);
		nn11 = r11;
		nn12 = r12;
		nn13 = r13;
		nn21 = r21;
		nn22 = r22;
		nn23 = r23;
		nn31 = r31;
		nn32 = r32;
		nn33 = r33;
		nn1t = t1;
		nn2t = t2;
		nn3t = t3;
		cout << "Joint 2 position is" << endl;
		cout << "(" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")" << endl;

		cout << "Your T1-2 looks like" << endl;
		vecal(dd2, aa2, a2, d2, 0, 0, 0);

		MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

		MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);

		cout << "Joint 3 position is" << endl;

		cout << "(" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")" << endl;

		nn11 = n11;
		nn12 = n12;
		nn13 = n13;
		nn21 = n21;
		nn22 = n22;
		nn23 = n23;
		nn31 = n31;
		nn32 = n32;
		nn33 = n33;
		nn1t = n1t;
		nn2t = n2t;
		nn3t = n3t;

		cout << "Your T2-3 looks like" << endl;
		vecal(dd3, aa3, a3, d3, 0, 0, 0);

		MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

		MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);

		cout << "Joint 4 position is" << endl;

		cout << "(" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")" << endl;

		nn11 = n11;
		nn12 = n12;
		nn13 = n13;
		nn21 = n21;
		nn22 = n22;
		nn23 = n23;
		nn31 = n31;
		nn32 = n32;
		nn33 = n33;
		nn1t = n1t;
		nn2t = n2t;
		nn3t = n3t;

		cout << "Your T3-4 looks like" << endl;
		vecal(dd4, aa4, a4, d4, 0, 0, 0);

		MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);


		MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);

		cout << "Joint end-effector position is" << endl;

		cout << "(" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")" << endl;

	}

	if (choice == 2)
	{
		cout << std::fixed;
		int ii = 0;
		cout << "Please enter present end-effector position (x,y,z)" << endl;
		cout << "x = ";
		cin >> prx;
		cout << "y = ";
		cin >> pry;
		cout << "z = ";
		cin >> prz;
		cout << "Please enter final end-effector position (x,y,z)" << endl;
		cout << "x = ";
		cin >> enx;
		cout << "y = ";
		cin >> eny;
		cout << "z = ";
		cin >> enz;
		cout << "You are moving from(" << prx << "," << pry << "," << prz << ") to (" << enx << "," << eny << "," << enz << ")." << endl;
		cout << "Press e to confirm, other characters to restart" << endl;
		cin >> loopc;
		if (loopc != 'e')
		{
			return;
		}
		prx = prx - inx;
		pry = pry - iny;
		prz = prz - inz;
		enx = enx - inx;
		eny = eny - iny;
		enz = enz - inz;
		cout << "Choice your 1/delta-x (larger number gives smoother movement, suggest 100)" << endl;
		cin >> deltax;
		trax = (enx - prx) / deltax;
		tray = (eny - pry) / deltax;
		traz = (enz - prz) / deltax;
		cout << endl << "<" << ii << ">  Joint 2          joint 3          joint 4          end-effector" << endl;
		while (deltax>0)
		{
			prx = prx + trax;
			pry = pry + tray;
			prz = prz + traz;
			enx = prx;
			eny = pry;
			enz = prz;
			dd1 = atan(eny / enx) * 180 / PI;
			if ((eny < 0 && enx<0) || (eny>0 && enx<0))
			{
				dd1 = 180 + dd1;
			}
			d1 = 20 * sin(45 * PI / 180);
			a1 = 20 * sin(45 * PI / 180);
			aa1 = -90;

			vecalw(dd1, aa1, a1, d1, 0, 0, 0);

			lp1x = xx;
			lp1y = yy;
			lp1z = zz;

			maxdistance = 20 * cos(45 * PI / 180) + 40 + 20 + 20;
			tdistance = sqrt((enx - 0)*(enx - 0) + (eny - 0)*(eny - 0) + (enz - 0)*(enz - 0));

			if (tdistance > maxdistance)
			{
				cout << "Error! too far for robotic arm to reach, max distance is " << maxdistance << endl;
				return;

			}


			redistance = sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y) + (enz - lp1z)*(enz - lp1z));



			if ((enx*enx + eny*eny) > (lp1x*lp1x + lp1y*lp1y))
			{
				if (redistance < 40)
				{
					dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
					dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
					dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;
				}
				else if (redistance == 40)
				{
					dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
					dd3 = 180;
					dd4 = 180;
				}
				else if (redistance > 40)
				{
					dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
					dd3 = acos((redistance - 40) / 2 / 20) * 180 / PI;
					dd4 = -2 * acos((redistance - 40) / 2 / 20) * 180 / PI;
				}

			}
			if ((enx*enx + eny*eny) < (lp1x*lp1x + lp1y*lp1y))
			{
				if (enz < 14.1421)
				{
					dd2 = 180 - (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180);

				}
				if (enz > 14.1421)
				{
					dd2 = (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180) - 90;

				}
				dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
				dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;

			}

			//fixed link 2 variable
			d2 = 0;
			a2 = 40;
			aa2 = 0;
			//fixed link 3 variable
			d3 = 0;
			a3 = 20;
			aa3 = 0;
			//fixed link 4 variable
			d4 = 0;
			a4 = 20;
			aa4 = 0;

			vecalw(dd1, aa1, a1, d1, 0, 0, 0);
			nn11 = r11;
			nn12 = r12;
			nn13 = r13;
			nn21 = r21;
			nn22 = r22;
			nn23 = r23;
			nn31 = r31;
			nn32 = r32;
			nn33 = r33;
			nn1t = t1;
			nn2t = t2;
			nn3t = t3;


			cout << "  (" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")";


			vecalw(dd2, aa2, a2, d2, 0, 0, 0);

			MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

			MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);


			cout << "   (" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")";

			nn11 = n11;
			nn12 = n12;
			nn13 = n13;
			nn21 = n21;
			nn22 = n22;
			nn23 = n23;
			nn31 = n31;
			nn32 = n32;
			nn33 = n33;
			nn1t = n1t;
			nn2t = n2t;
			nn3t = n3t;


			vecalw(dd3, aa3, a3, d3, 0, 0, 0);

			MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

			MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);



			cout << "   (" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")";

			nn11 = n11;
			nn12 = n12;
			nn13 = n13;
			nn21 = n21;
			nn22 = n22;
			nn23 = n23;
			nn31 = n31;
			nn32 = n32;
			nn33 = n33;
			nn1t = n1t;
			nn2t = n2t;
			nn3t = n3t;

			vecalw(dd4, aa4, a4, d4, 0, 0, 0);

			MatrixTMatrix(nn11, nn12, nn13, nn1t, nn21, nn22, nn23, nn2t, nn31, nn32, nn33, nn3t, r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);


			MatrixTVec(n11, n12, n13, n1t, n21, n22, n23, n2t, n31, n32, n33, n3t, 0, 0, 0);



			cout << "   (" << xx + inx << ", " << yy + iny << ", " << zz + inz << ")" << endl;



			deltax--;
		}













	}

	if (choice == 30000)
	{

		cout << "Please enter present end-effector position (x,y,z)" << endl;
		cout << "x = ";
		cin >> prx;
		cout << "y = ";
		cin >> pry;
		cout << "z = ";
		cin >> prz;


		nd1 = atan(pry / prx) * 180 / PI;

		d1 = 20 * sin(45 * PI / 180);
		a1 = 20 * sin(45 * PI / 180);
		aa1 = -90;

		vecal(nd1, aa1, a1, d1, 0, 0, 0);

		lp1x = xx;
		lp1y = yy;
		lp1z = zz;

		maxdistance = 20 * cos(45 * PI / 180) + 40 + 20 + 20;
		tdistance = sqrt((prx - inx)*(prx - inx) + (pry - iny)*(pry - iny) + (prz - inz)*(prz - inz));

		if (tdistance > maxdistance)
		{
			cout << "Error! too far for robotic arm to reach, max distance is " << maxdistance << endl;
			return;

		}


		redistance = sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y) + (prz - lp1z)*(prz - lp1z));



		if ((prx*prx + pry*pry) > (lp1x*lp1x + lp1y*lp1y))
		{
			if (redistance < 40)
			{
				nd2 = -atan((prz - lp1z) / sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y))) / PI * 180;
				nd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
				nd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;
			}
			else if (redistance == 40)
			{
				nd2 = -atan((prz - lp1z) / sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y))) / PI * 180;
				nd3 = 180;
				nd4 = 180;
			}
			else if (redistance > 40)
			{
				nd2 = -atan((prz - lp1z) / sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y))) / PI * 180;
				nd3 = acos((redistance - 40) / 2 / 20) * 180 / PI;
				nd4 = -2 * acos((redistance - 40) / 2 / 20) * 180 / PI;
			}

		}
		if ((prx*prx + pry*pry) < (lp1x*lp1x + lp1y*lp1y))
		{
			if (prz < 14.1421)
			{
				nd2 = 180 - (-atan((prz - lp1z) / sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y))) / PI * 180);

			}
			if (prz > 14.1421)
			{
				nd2 = (-atan((prz - lp1z) / sqrt((prx - lp1x)*(prx - lp1x) + (pry - lp1y)*(pry - lp1y))) / PI * 180) - 90;

			}
			nd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
			nd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;

		}


		cout << "Four link angles are: " << endl;
		cout << nd1 << "," << nd2 << "," << nd3 << "," << nd4 << "," << endl;


		cout << "Please enter final end-effector position (x,y,z)" << endl;
		cout << "x = ";
		cin >> enx;
		cout << "y = ";
		cin >> eny;
		cout << "z = ";
		cin >> enz;


		dd1 = atan(eny / enx) * 180 / PI;

		d1 = 20 * sin(45 * PI / 180);
		a1 = 20 * sin(45 * PI / 180);
		aa1 = -90;

		vecal(dd1, aa1, a1, d1, 0, 0, 0);

		lp1x = xx;
		lp1y = yy;
		lp1z = zz;

		maxdistance = 20 * cos(45 * PI / 180) + 40 + 20 + 20;
		tdistance = sqrt((enx - inx)*(enx - inx) + (eny - iny)*(eny - iny) + (enz - inz)*(enz - inz));

		if (tdistance > maxdistance)
		{
			cout << "Error! too far for robotic arm to reach, max distance is " << maxdistance << endl;
			return;

		}


		redistance = sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y) + (enz - lp1z)*(enz - lp1z));



		if ((enx*enx + eny*eny) > (lp1x*lp1x + lp1y*lp1y))
		{
			if (redistance < 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
				dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;
			}
			else if (redistance == 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = 180;
				dd4 = 180;
			}
			else if (redistance > 40)
			{
				dd2 = -atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180;
				dd3 = acos((redistance - 40) / 2 / 20) * 180 / PI;
				dd4 = -2 * acos((redistance - 40) / 2 / 20) * 180 / PI;
			}

		}
		if ((enx*enx + eny*eny) < (lp1x*lp1x + lp1y*lp1y))
		{
			if (enz < 14.1421)
			{
				dd2 = 180 - (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180);

			}
			if (enz > 14.1421)
			{
				dd2 = (-atan((enz - lp1z) / sqrt((enx - lp1x)*(enx - lp1x) + (eny - lp1y)*(eny - lp1y))) / PI * 180) - 90;

			}
			dd3 = 180 - acos((40 - redistance) / 2 / 20) * 180 / PI;
			dd4 = 2 * acos((40 - redistance) / 2 / 20) * 180 / PI;

		}


		cout << "Four link angles are: " << endl;
		cout << dd1 << "," << dd2 << "," << dd3 << "," << dd4 << "," << endl;






		cout << "You are moving from(" << prx << "," << pry << "," << prz << ") to (" << enx << "," << eny << "," << enz << ")." << endl;
		cout << "Press e to confirm, other characters to restart" << endl;
		cin >> loopc;
		if (loopc != 'e')
		{
			return;
		}
		cout << "Choice your delta-x (smaller x generate smoother movement)" << endl;
		cin >> deltax;


	}


}
void vecal(double wc, double xc, double yc, double zc, double sx, double sy, double sz)
{
	ddi = wc;
	aai = xc;
	ai = yc;
	di = zc;


	r11 = cos(ddi*PI / 180);
	r21 = sin(ddi*PI / 180);
	r31 = Z;

	r12 = -sin(ddi*PI / 180)*cos(aai*PI / 180);
	r22 = cos(ddi*PI / 180)*cos(aai*PI / 180);
	r32 = sin(aai*PI / 180);

	r13 = sin(ddi*PI / 180)*sin(aai*PI / 180);
	r23 = -cos(ddi*PI / 180)*sin(aai*PI / 180);
	r33 = cos(aai*PI / 180);

	t3 = di;
	t2 = ai*sin(ddi*PI / 180);
	t1 = ai*cos(ddi*PI / 180);

	cout << "--                                            --" << endl;
	cout << "|                                              |" << endl;
	cout << std::fixed;
	cout << "| " << r11 << ",  " << r12 << ",  " << r13 << " | " << t1 << "  |" << endl;
	cout << "| " << r21 << ",  " << r22 << ",  " << r23 << " | " << t2 << "  |" << endl;
	cout << "| " << r31 << ",  " << r32 << ",   " << r33 << " | " << t3 << "  |" << endl;
	cout << "----------------------------------------" << endl;
	cout << "| " << Z << ",  " << Z << ",  " << Z << " | " << o << "   |" << endl;
	cout << "|                                              |" << endl;
	cout << "--                                            --" << endl;
	cout << endl;
	cout << endl;

	xx = r11*sx + r12*sy + r13*sz + t1;
	yy = r21*sx + r22*sy + r23*sz + t2;
	zz = r31*sx + r32*sy + r33*sz + t3;

	return;
}
void MatrixTMatrix(double rr11, double rr12, double rr13, double rt1, double rr21, double rr22, double rr23, double rt2, double rr31, double rr32, double rr33, double rt3, double nr11, double nr12, double nr13, double nt1, double nr21, double nr22, double nr23, double nt2, double nr31, double nr32, double nr33, double nt3)
{
	n11 = rr11*nr11 + rr12*nr21 + rr13*nr31;
	n12 = rr11*nr12 + rr12*nr22 + rr13*nr32;
	n13 = rr11*nr13 + rr12*nr23 + rr13*nr33;
	n1t = rr11*nt1 + rr12*nt2 + rr13*nt3 + rt1;

	n21 = rr21*nr11 + rr22*nr21 + rr23*nr31;
	n22 = rr21*nr12 + rr22*nr22 + rr23*nr32;
	n23 = rr21*nr13 + rr22*nr23 + rr23*nr33;
	n2t = rr21*nt1 + rr22*nt2 + rr23*nt3 + rt2;

	n31 = rr31*nr11 + rr32*nr21 + rr33*nr31;
	n32 = rr31*nr12 + rr32*nr22 + rr33*nr32;
	n33 = rr31*nr13 + rr32*nr23 + rr33*nr33;
	n3t = rr31*nt1 + rr32*nt2 + rr33*nt3 + rt3;

	return;
}
void MatrixTVec(double rr11, double rr12, double rr13, double rt1, double rr21, double rr22, double rr23, double rt2, double rr31, double rr32, double rr33, double rt3, double npx, double npy, double npz)
{
	xx = rr11*npx + rr12*npy + rr13*npz + rt1;
	yy = rr21*npx + rr22*npy + rr23*npz + rt2;
	zz = rr31*npx + rr32*npy + rr33*npz + rt3;
	return;

}

void vecalw(double wc, double xc, double yc, double zc, double sx, double sy, double sz)
{
	ddi = wc;
	aai = xc;
	ai = yc;
	di = zc;


	r11 = cos(ddi*PI / 180);
	r21 = sin(ddi*PI / 180);
	r31 = Z;

	r12 = -sin(ddi*PI / 180)*cos(aai*PI / 180);
	r22 = cos(ddi*PI / 180)*cos(aai*PI / 180);
	r32 = sin(aai*PI / 180);

	r13 = sin(ddi*PI / 180)*sin(aai*PI / 180);
	r23 = -cos(ddi*PI / 180)*sin(aai*PI / 180);
	r33 = cos(aai*PI / 180);

	t3 = di;
	t2 = ai*sin(ddi*PI / 180);
	t1 = ai*cos(ddi*PI / 180);


	xx = r11*sx + r12*sy + r13*sz + t1;// position x
	yy = r21*sx + r22*sy + r23*sz + t2;// position y
	zz = r31*sx + r32*sy + r33*sz + t3;

	return;
}