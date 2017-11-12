class Odometry {
private:
    double x = 0;
    double y = 0;
    double th = 0;

    double vx = 0;
    double vy = 0;
    double vth = 0;

    double wheelbase;

public:
    Odometry(double wheelbase);

    double update(double deltaMetersLeft, double deltaMetersRight, double dt);

    double getX() {return x;}
    double getY() {return y;}
    double getTh() {return th;}

    double getVX() {return vx;}
    double getVY() {return vy;}
    double getVTh() {return vth;}
};
