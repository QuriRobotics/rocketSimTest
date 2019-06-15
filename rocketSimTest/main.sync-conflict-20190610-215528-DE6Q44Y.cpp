#include <iostream>
#include <Windows.h>

#define x 0
#define y 1
#define z 2

#define f 0.001

class PID
{
public:
	double kp, ki, kd, buff, err, errold, u;
	int buffMax = 1000;

	double update(double input, double goal)
	{
		err = goal - input;
		buff += err;
		u = err * kp + (err - errold)*kd + buff * ki;
		errold = err;
		return u;
	}

	PID(double p, double i, double d)
	{
		this->kp = p;
		this->ki = i;
		this->kd = d;
		buff = 0, err = 0, errold = 0, u = 0;
	}
	PID(){}
};

class Rocket
{
public:
	double coor[3];
	double vel[3];
	double thrust = 0;
	double mass = 1;

	double kp = 1, ki = 0, kd = 10;

	PID pid;
	
	void update(double goal)
	{
		thrust = max(pid.update(coor[z], goal), 0);
		vel[z] += (thrust / mass)*f;
		coor[x] += vel[x] * f;
		coor[y] += vel[y] * f;
		coor[z] += vel[z] * f;
		if (coor[z] < 0)
		{
			coor[z] = 0;
			vel[z] = 0;
		}
	}

	bool onGround()
	{
		return coor[z] == 0;
	}
	Rocket(double xCoor, double yCoor, double zCoor, double xVel, double yVel, double zVel)
	{
		this->coor[x] = xCoor;
		this->coor[y] = yCoor;
		this->coor[z] = zCoor;
		this->vel[x] = xVel;
		this->vel[y] = yVel;
		this->vel[z] = zVel;
		pid.kp = kp;
		pid.ki = ki;
		pid.kd = kd;
		update(0);
	}
	/*Rocket(double xCoor, double yCoor, double zCoor)
	{
		this->coor[x] = xCoor;
		this->coor[y] = yCoor;
		this->coor[z] = zCoor;
		this->vel[x] = 0;
		this->vel[y] = 0;
		this->vel[z] = 0;
		PID pid(kp, ki, kd);
		update(0);
	}
	Rocket()
	{
		this->coor[x] = 0;
		this->coor[y] = 0;
		this->coor[z] = 0;
		this->vel[x] = 0;
		this->vel[y] = 0;
		this->vel[z] = 0;
		PID pid(kp, ki, kd);
		update(0);
	}*/
};

using namespace std;

int main()
{
	double g = -9.81;

	int counter = 0;

	Rocket r(0, 0, 10, 0, 0, 0);

	while (!r.onGround())
	{
		r.vel[z] += g * f;

		//if (r.coor[z] < 70) r.thrust = (70-r.coor[z] ) * 1;
		//else r.thrust = 0;

		r.update(50);

		if (counter % 100 == 0)
		{
			for (int i = 0; i < r.coor[z] / 3 - r.thrust / 10; i++)
			{
				cout << " ";
			}
			for (int i = 0; i < r.thrust / 10; i++)
			{
				cout << "#";
			}
			cout << ">=" << int(r.thrust) << "=>\n";
			//cout << "T+" << counter*f << "\t" << r.coor[z] << "\t" << r.vel[z] << endl;
		}

		Sleep(1000 * f);
		counter++;
	}

	return 0;
}