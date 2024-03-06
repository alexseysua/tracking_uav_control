#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <limits>

#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <ctime>
#include <cstdlib>
#include <chrono>

float rad2Deg = 180 / M_PI;
float Deg2Rad = M_PI / 180;

float twoDecimal(float num)
{
    float new_num = floor(num * 100.0) / 100.0;
    return new_num;
}

float dotProduct(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    float result = 0;
    for(int i = 0; i < v1.size(); ++i) 
    {
        result += v1(i) * v2(i);
    }
    
    return result;
}

float vectorMagnitude(Eigen::Vector3f v)
{
    float result = sqrt( v.x()*v.x() + v.y()*v.y() + v.z()*v.z() );
    return result;
}

Eigen::Matrix3f rotationMatrix(char axis, float theta)
{
    Eigen::Matrix3f R;

    switch(axis)
    {
        case 'Z':
            R << cos(theta), sin(theta), 0,
                  -sin(theta), cos(theta), 0,
                  0, 0, 1;
            break;
        case 'Y':
            R << cos(theta), 0, -sin(theta), 
                  0, 1, 0,
                  sin(theta), 0, cos(theta);
            break;
        case 'X':
            R << 1, 0, 0, 
                  0, cos(theta), sin(theta), 
                  0, -sin(theta), cos(theta);
            break;
        default:
            break; 
    }
    
    return R;
}

Eigen::AngleAxisf Quaternion2AngleAxis(Eigen::Quaternionf q)
{
    q = q.normalized();
    // angleAxis
    float angle = 2 * acos( q.w() );
    float x = q.x() / sqrt( 1 - q.w()*q.w() );
    float y = q.y() / sqrt( 1 - q.w()*q.w() );
    float z = q.z() / sqrt( 1 - q.w()*q.w() );
    // rotation matrix
    Eigen::Vector3f axis(x, y, z);
    Eigen::AngleAxisf rot = Eigen::AngleAxisf( angle, axis );

    return rot;
}

Eigen::Vector3f Quaternion2Euler(Eigen::Quaternionf q)
{
    q = q.normalized();

    float roll = std::atan2( 2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()) );

    float pitch = 0;
    float sinp = 2*( q.w()*q.y()-q.z()*q.x() );
    if( std::abs(sinp) >= 1 )
    {
        pitch = std::copysign( M_PI/2, sinp);
    }
    else 
    {
        pitch = std::asin(sinp);
    }

    float yaw = std::atan2( 2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()) );
    Eigen::Vector3f euler(roll, pitch, yaw);

    return euler;
}

Eigen::Quaternionf Quaternion2Euler(Eigen::Vector3f euler)
{
    Eigen::Quaternionf q;
    double roll = euler[0], pitch = euler[1], yaw = euler[2];

    /*** order: Z-Y-X ***/
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    return q;
}

Eigen::Matrix3f Euler2RotaMatrix(Eigen::Vector3f euler)
{
    Eigen::Matrix3f R = rotationMatrix('X', euler.x()) * rotationMatrix('Y', euler.y()) * rotationMatrix('Z', euler.z());
    return R;
}

Eigen::Matrix3f Quat2RotaMatrix(Eigen::Quaternionf q)
{
    Eigen::Matrix3f R = Euler2RotaMatrix( Quaternion2Euler(q) );
    return R;
}

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) 
{
    return (epsilon > std::abs(a - b));
}

Eigen::Vector3f RotaM2Euler(Eigen::Matrix3f R)
{
    Eigen::Vector3f euler;
    //check for gimbal lock
    if (closeEnough(R.coeff(0, 2), -1.0f)) {
        euler.x() = 0; //gimbal lock, value of x doesn't matter
        euler.y() = M_PI / 2;
        euler.z() = euler.x() + atan2( R.coeff(1, 0), R.coeff(2, 0) );
        return euler;
    } else if (closeEnough(R.coeff(0, 2), 1.0f)) {
        euler.x() = 0;
        euler.y() = -M_PI / 2;
        euler.z() = -euler.x() + atan2(-R.coeff(1, 0), -R.coeff(2, 0));
        return euler;
    } else { //two solutions exist
        float x1 = -asin(R.coeff(0, 2));
        float x2 = M_PI - x1;

        float y1 = atan2(R.coeff(1, 2) / cos(x1), R.coeff(2, 2) / cos(x1));
        float y2 = atan2(R.coeff(1, 2) / cos(x2), R.coeff(2, 2) / cos(x2));

        float z1 = atan2(R.coeff(0, 1) / cos(x1), R.coeff(0, 0) / cos(x1));
        float z2 = atan2(R.coeff(0, 1) / cos(x2), R(0, 0) / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            euler.x() = x1;
            euler.y() = y1;
            euler.z() = z1;
            return euler;
        } else {
            euler.x() = x2;
            euler.y() = y2;
            euler.z() = z2;
            return euler;
        }
    }
}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

class TicToc
{
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;

public:
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        double elapsed_milli = std::chrono::duration<double, std::milli> (end - start).count();
        return elapsed_milli;
    }
};