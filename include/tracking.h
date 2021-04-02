#ifndef _TRACKING_H_
#define _TRACKING_H_

#include "math.h"
#define _USE_MATH_DEFINES
#include "odom.h"

#define WHEELBASE 10
#define BACK_WHEEL_OFFSET 6

#define TRACKING_WHEEL_DIAMETER 2.75f
#define TRACKING_WHEEL_DEGREE_TO_INCH (M_PI * TRACKING_WHEEL_DIAMETER / 360)

double radToDeg(double r);
double degToRad(double d);

typedef struct Vector2 {
private:
	double x, y;

public:
	Vector2(double _x, double _y);
	Vector2();

	double getX();
	double getY();

	double getMagnitude();
	double getAngle();

	void normalize();

	// friend keyword allows access to private members
	friend Vector2 operator + (const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator - (const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator * (const Vector2 &v1, double scalar);
} Vector2;

typedef struct TrackingData {
private:
	double heading;
	Vector2 pos;

public:
	TrackingData(double _x, double _y, double _h);

	double getX();
	double getY();
	double getHeading();
	Vector2 getPos();

	void update(double _x, double _y, double _h);

} TrackingData;

/*
	Tracks encoder positions; No parameters
*/
void tracking();
Vector2 rotateVector(Vector2 vec, double angle);
Vector2 toLocalCoordinates(Vector2 vec);
extern TrackingData trackingData;

typedef struct VirtualEncoder {
private:
    double ticks;
    double offset; // Positive = right
    bool lateral;

public:
    VirtualEncoder(double _offset, bool _lateral = false);
	VirtualEncoder();
    void reset();
    long int read();
    void update(Vector2 dP, double dO); // params in local space

} VirtualEncoder;

Vector2 glmToCustom(glm::vec2 v);
glm::vec2 customToGLM(Vector2 v);

extern VirtualEncoder leftTrackingWheel;
extern VirtualEncoder rightTrackingWheel;
extern VirtualEncoder backTrackingWheel;
extern XDrive chassis;

#endif
