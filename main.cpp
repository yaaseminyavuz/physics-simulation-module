#include <iostream>
#include <cmath>
#include <cctype>

using namespace std;
const float g = 9.8;
const float PI = 3.14159;
const float G = 6.6743 * pow(10, -11);

void menu();

// Air resistance constants
const double air_density = 1.225; // Air density
const double Cd = 0.47; // Drag coefficient

// Function to compute air resistance
double resistanceForce(double v, double A) {
    return 0.5 * Cd * A * air_density * v * v;
}

// Function to validate and get positive inputs
double getPositiveInput(string prompt) {
    double value;
    do {
        cout << prompt;
        cin >> value;
        if (value <= 0) {
            cout << "Oops! The value must be positive. Please try again." << endl;
        }
    } while (value <= 0); // Repeat until we get a positive value
    return value;
}

//Function to validate and get angle input between 0 and 90
double getValidAngle(string prompt) {
    double value;
    do {
        cout << prompt;
        cin >> value;
        if (value <= 0 || value >= 90) {
            cout << "Oops! The value must be between 0 and 90. Please try again." << endl;
        }
    } while (value <= 0 || value >= 90); // Repeat until we get a positive value
    return value;
}

// Function to check if the projectile hits a person (no air resistance)
void checkHitNoAirResistance(double impactX, double initialHeight, double initialSpeed, double vxInitial, double vyInitial) {
    string hitCheck;
    do {
        cout << "Do you want to check if the projectile hits a person due to their location? (yes/no): ";
        cin >> hitCheck;
        if (hitCheck != "yes" && hitCheck != "no") {
            cout << "Please type just 'yes' or 'no'!!!" << endl;
        }
    } while (hitCheck != "yes" && hitCheck != "no");// to get answer just yes or no

    if (hitCheck == "yes") {
        double xPerson = getPositiveInput("Enter the horizontal position of the person (in m): ");
        double yPerson = getPositiveInput("Enter the height of the person (in m): ");

        // Check if the person is within the horizontal range
        if (xPerson > impactX) {
            cout << "The person is out of range. The projectile won't hit them." << endl;
        }
        else {
            double tHit = xPerson / vxInitial; // Time at which projectile would hit person's x position
            double yHit = initialHeight + vyInitial * tHit - 0.5 * g * tHit * tHit;

            if (yHit <= yPerson) {
                cout << "Boom! The projectile hits the person!" << endl;
                cout << "Position at impact: X = " << xPerson << " m, Y = " << yHit << " m" << endl;
            }
            else {
                cout << "The projectile doesn't hit the person." << endl;
            }
        }
    }
}

// Function to check if the projectile hits a person (with air resistance)
void checkHitWithAirResistance(double initialHeight, double launchAngleRad, double impactX, double vx, double vy, double A, double m) {
    string hitCheck;
    do {
        cout << "Do you want to check if the projectile hits a person due to their location? (yes/no): ";
        cin >> hitCheck;
        if (hitCheck != "yes" && hitCheck != "no") {
            cout << "Please type just 'yes' or 'no'!!!" << endl;
        }
    } while (hitCheck != "yes" && hitCheck != "no");

    if (hitCheck == "yes") {
        double xPerson = getPositiveInput("Please enter the horizontal position of the person (in m): ");
        double yPerson = getPositiveInput("Please enter the height of the person (in m): ");

        // Check if the person is within the horizontal range
        if (xPerson > impactX) {
            cout << "The person is out of range. The projectile won't hit them." << endl;
        }
        else {  // Simulate the projectile's motion and check if it hits the person
            double x = 0.0;
            double y = initialHeight;
            double vxCurr = vx; // Current horizontal velocity
            double vyCurr = vy; // Current vertical velocity
            double t = 0.0;
            double dt = 0.0000001;
            double a_x;
            double a_y;

            while (x <= xPerson && y!= 0) {
                a_x = -(resistanceForce(vx, A) / m);
                vx += a_x * dt;
                x += vx * dt;
                a_y =- g +resistanceForce(vy, A) / m;
                vy += a_y * dt;
                y += vy * dt;
                t += dt;
            }

            // If the horizontal position of the projectile is at the person's position
            if (y <= yPerson) {
                cout << "Boom! The projectile hits the person!" << endl;
                cout << "Position at impact: X = " << xPerson << " m, Y = " << y << " m" << endl;
                return; // Exit the function once the projectile hits the person
            }

            // If the projectile does not hit the person
            cout << "The projectile doesn't hit the person." << endl;
        }
    }
}

// Simulate projectile motion without air resistance
void simulateNoAirResistance(double initialSpeed, double launchAngleRad, double initialHeight) {
    double vxInitial = initialSpeed * cos(launchAngleRad); // Horizontal velocity
    double vyInitial = -initialSpeed * sin(launchAngleRad); // Vertical velocity (negative because it is downward)
    double vy = vyInitial;

    double x = 0.0;
    double y = initialHeight; // Initial height
    double dt = 0.00001; // Time step
    double t = 0.0;

    cout << "\nTime (s)\tX (m)\tY (m)" << endl;

    // Simulate motion until projectile hits the ground
    while (initialHeight + vyInitial * t - 0.5 * g * t * t >= 0) {
        x += vxInitial * dt;  // Horizontal movement
        vy -= g * dt;  // Gravity's effect on vertical velocity()
        y += vyInitial * dt - 0.5 * g * dt * dt;  // Vertical movement
        t += dt;

        if ((int)(t * 100000) % 10000 == 0) {
            cout << floor(t * 10) / 10 << "\t\t" << x << "\t" << initialHeight + vyInitial * t - 0.5 * g * t * t << endl;
        }
    }

    cout << "\nFinal horizontal position at impact: x = " << x << " meters" << endl;
    checkHitNoAirResistance(x,initialHeight, initialSpeed, vxInitial, vyInitial);
}

// Simulate projectile motion with air resistance
void simulateWithAirResistance(double initialSpeed, double launchAngleRad, double A, double m, double initialHeight) {
    double vxInitial = initialSpeed * cos(launchAngleRad); // Horizontal velocity
    double vyInitial = -initialSpeed * sin(launchAngleRad); // Vertical velocity (negative because it is downward)
    double vx = vxInitial;
    double vy = vyInitial;
    double x = 0.0;
    double y = initialHeight; // Initial height
    double dt = 0.0000001; // Time step
    double t = 0.0;
    double a_x;
    double a_y;

    cout << "\nTime (s)\tX (m)\tY (m)" << endl;

    // Simulate motion until projectile hits the ground
    while (y >= 0) {

        // Update horizontal acceleration due to air resistance
        a_x = -(resistanceForce(vx, A) / m);
        vx += a_x * dt;
        x += vx * dt;

        // Update vertical acceleration due to gravity and air resistance
        a_y = - g + resistanceForce(vy, A) / m;
        vy += a_y * dt;
        y += vy * dt;

        t += dt;

        if ((int)(t * 10000000) % 1000000 == 0) {
            cout << floor(t * 10) / 10 << "\t\t" << x << "\t" << y << endl;
        }
    }

    cout << "\nFinal horizontal position at impact: x = " << x << " meters" << endl;
    checkHitWithAirResistance(initialHeight, launchAngleRad, x, vxInitial, vyInitial, A, m);
}

void motion() {
    cout << "=========================================================" << endl;
    cout << "        Welcome to the Projectile Motion Simulator        " << endl;
    cout << "=========================================================" << endl;

    cout << "\nthe aim of this application is to simulate motion of an object which is released from height of 14 meters\n";
    cout << "with a specific angle and to check if it hits a person which is at the platform\n";
    cout << "now to get started to calculations\n";

    string ans;
    do {

        // Get user input
        double initialSpeed = getPositiveInput("Please enter the initial speed (in m/s): ");
        double initialHeight = getPositiveInput("Please enter the initial Height (in m): ");
        double launchAngle = getValidAngle("Please enter the launch angle (in degrees): ");
        double launchAngleRad = launchAngle * PI / 180.0; // Convert degrees to radians

        // Ask user if air resistance should be included
        string includeAirResistance;
        do {
            cout << "Do you want to include air resistance? (yes/no): ";
            cin >> includeAirResistance;
            if (includeAirResistance != "yes" && includeAirResistance != "no") {
                cout << "Please type just 'yes' or 'no'!!!" << endl;
            }
        } while (includeAirResistance != "yes" && includeAirResistance != "no");

        if (includeAirResistance == "yes") {
            double A = getPositiveInput("Please enter the cross-sectional area (A) (in m^2): ");
            double m = getPositiveInput("Enter the mass of object (in kg): ");

            simulateWithAirResistance(initialSpeed, launchAngleRad, A, m, initialHeight);
        }
        else {
            simulateNoAirResistance(initialSpeed, launchAngleRad, initialHeight);
        }
        cout << "\nnow if you want to make another calculation please enter 'yes' or 'no' to go back to menu\n";
        cin >> ans;
        for (auto& c : ans) c = tolower(c);

        while (ans != "yes" && ans != "no") {
            cout << "please enter 'yes' or 'no'\n";
            std::cin >> ans;
            for (auto& c : ans) c = tolower(c);
        }

        system("cls");

    } while (ans == "yes");

    menu();
}

// Function to calculate final velocities and kinetic energy loss
void calculateVelocities(double vAi, double thetaA, double thetaB, double& vAf, double& vBf, double& energyLossFraction) {
    // Convert angles to radians
    thetaA = thetaA * PI / 180.0;
    thetaB = thetaB * PI / 180.0;

    // Calculate final velocities
    vAf = vAi * (sin(thetaB) / sin(thetaA + thetaB));
    vBf = vAf * (sin(thetaA)) / sin(thetaB);

    // Kinetic energy calculations
    double KE_initial = 0.5 * vAi * vAi;  // Initial kinetic energy of asteroid A
    double KE_final = 0.5 * (vAf * vAf);  // Final kinetic energy of both asteroids
    energyLossFraction = (KE_initial - KE_final) / KE_initial;  // Fraction of energy lost
}

// Function to print trajectory data
void printTrajectories(double vAf, double thetaA, double vBf, double thetaB) {
    // Convert angles to radians
    thetaA = thetaA * PI / 180.0;
    thetaB = thetaB * PI / 180.0;

    // Compute x and y components of velocities
    double xA = vAf * cos(thetaA);
    double yA = vAf * sin(thetaA);
    double xB = vBf * cos(thetaB);
    double yB = vBf * sin(thetaB);

    // Display trajectory data
    cout << "Trajectory Data (x, y):" << std::endl;
    cout << "Asteroid A: (" << xA << ", " << yA << ")" << std::endl;
    cout << "Asteroid B: (" << xB << ", " << yB << ")" << std::endl;
}

void collision() {

    cout << "=========================================================" << endl;
    cout << "        Welcome to the Asteroid Collision Simulator       " << endl;
    cout << "=========================================================" << endl;

    cout << "\nthe aim of this application is to simulate two asteroids after they collide\n";
    cout << "and to calculate their velocities and energy loss fraction of the asteroid which collides the other one\n";
    cout << "now to get started to calculations\n";

    string repeat;  // Initialize repeat character
    do {
        // Variables
        double vAi, thetaA, thetaB, vAf, vBf, energyLossFraction;

        // Input from user
        vAi = getPositiveInput("Please enter the initial speed of Asteroid A (in m/s): ");
        thetaA = getValidAngle("Please enter the deflection angle of Asteroid A (in degrees): ");
        thetaB = getValidAngle("Please enter the angle of Asteroid B after the collision (in degrees): ");

        // Perform calculations
        calculateVelocities(vAi, thetaA, thetaB, vAf, vBf, energyLossFraction);

        // Display results
        cout << "\nResults:" << std::endl;
        cout << "Final velocity of Asteroid A: " << vAf << " m/s" << std::endl;
        cout << "Final velocity of Asteroid B: " << vBf << " m/s" << std::endl;
        cout << "Fraction of kinetic energy dissipated: " << energyLossFraction * 100 << " %" << std::endl;

        // Print trajectory data
        printTrajectories(vAf, thetaA, vBf, thetaB);

        // Repeat the operation?
        cout << "\nnow if you want to make another calculation please enter 'yes' or 'no' to go back to menu\n";
        cin >> repeat;
        for (auto& c : repeat) c = tolower(c);
        system("cls");
        while (repeat != "yes" && repeat != "no") {
            cout << "please enter 'yes' or 'no'\n";
            std::cin >> repeat;
            for (auto& c : repeat) c = tolower(c);
        }

    } while (repeat == "yes");
    menu();

}

void unit_conversion1(double& value) {
    //g to kg
    value = value / 1000;
}

void unit_conversion2(double& value) {
    //cm to m
    value = value / 100;
}

double max_angle(double maxHeight, double lengthPendulum) {
    double ratio = 1 - (maxHeight / lengthPendulum);

    // checks if the value is valid
    if (ratio < -1.0 || ratio > 1.0) {
        cout << "\a";  // alarm sounds if the value is invalid
        cout << "Error: Invalid ratio (" << ratio << "). The ratio must be between -1 and 1." << endl;
        return -1; // -1 is returned on error
    }
    return acos(ratio);
}

void pendulum() {

    cout << "=========================================================" << endl;
    cout << "        Welcome to the Ballistic Pendulum Simulator        " << endl;
    cout << "=========================================================" << endl;

    cout << "\nthe aim of this application is to calculate the change over heigh of a pendulum when it gets hit by a bullet\n";
    cout << "initial kinetic energy of bullet and kinetic energy of combined body at collision moment will be printed as output as well\n";
    cout << "now to get started to calculations\n";


    string ans = "yes";
    while (ans == "yes") {
        double bulletMass, bulletVelocity, pendulumMass, lengthPendulum, finalVelocity, maxHeight, KE_bullet_initial, KE_system_after, maxAngle, period;

        cout << endl;

        // we get the required values from the user
        bulletMass = getPositiveInput("Please enter mass of bullet (in g): ");
        bulletVelocity = getPositiveInput("Please enter velocity of bullet (in m/s): ");
        pendulumMass = getPositiveInput("Please enter mass of pendulum (in g): ");
        lengthPendulum = getPositiveInput("Please enter length of pendulum (in cm): ");

        // we make unit conversions
        unit_conversion1(bulletMass); // kg
        unit_conversion1(pendulumMass); // kg
        unit_conversion2(lengthPendulum); // m

        // post collision speed and height calculations
        KE_bullet_initial = 0.5 * bulletMass * pow(bulletVelocity, 2);
        finalVelocity = (bulletMass * bulletVelocity) / (bulletMass + pendulumMass);
        maxHeight = (pow(finalVelocity, 2)) / (2 * g);
        KE_system_after = 0.5 * (bulletMass + pendulumMass) * pow(finalVelocity, 2);
        period = 2 * PI * sqrt(lengthPendulum / g);

        // we calculate the maximum angle
        maxAngle = max_angle(maxHeight, lengthPendulum);

        // error status check
        if (maxAngle == -1) {
            cout << "\aCalculation failed due to invalid ratio." << endl;
            continue;
            // Program terminates in case of error
        }

        // we print the results on the screen
        cout << "Final velocity: " << finalVelocity << " m/s" << endl;
        cout << "Maximum height: " << maxHeight << " m" << endl;
        cout << "Initial kinetic energy of the bullet: " << KE_bullet_initial << " J" << endl;
        cout << "Kinetic energy of the system after collision: " << KE_system_after << " J" << endl;
        cout << "Maximum angle: " << maxAngle << " radians (" << maxAngle * (180.0 / PI) << " degrees)" << endl;
        cout << "Period of the pendulum: " << period << " s" << endl; cout << endl;


        cout << "\nnow if you want to make another calculation please enter 'yes' or 'no' to go back to menu\n";
        cin >> ans;
        for (auto& c : ans) c = tolower(c);

        while (ans != "yes" && ans != "no") {
            cout << "please enter 'yes' or 'no'\n";
            std::cin >> ans;
            for (auto& c : ans) c = tolower(c);
        }

        system("cls");

    }

    menu();
}

//gives the velocity of satellite
float velocitycalculation(float radius, float altitude, float mass) {

    return sqrt((mass * G) / (altitude + radius));
}

//gives the period of revolution of satellite
float periodcalculation(float radius, float altitude, float mass) {

    return sqrt((4 * PI * PI * pow((altitude + radius), 3)) / (G * mass));
}

//4th application
void satellite() {

    cout << "=========================================================" << endl;
    cout << "        Welcome to the Satellite Motion Simulator        " << endl;
    cout << "=========================================================" << endl;

    const int altitude_from_earth = 600000;
    const float mass_of_world = 5.972 * pow(10, 24);
    const int radius_of_world = 6378137;
    float altitude;
    float radius;
    float mass;
    string ans;

    float velocity;
    float period;

    //an introduction message to explain the aim of application and how it works
    cout << "\nthe aim of this application is to derive required velocity and an orbital period value for a satellite\n";
    cout << "to revolve around an orbit based on defined mass value of orbit and altitude of satellite from the orbit's surface\n";
    cout << "for example: a satellite which is revolving around the earth from 600km altitude has to revolve with a velocity of ";
    cout << velocitycalculation(radius_of_world, altitude_from_earth, mass_of_world) << " m/s in " << periodcalculation(radius_of_world, altitude_from_earth, mass_of_world) << " seconds\n";


    do {
        //ask user for required inputs

        mass = getPositiveInput("Please enter the mass value of the orbit(in kg):\n");
        radius = getPositiveInput("Please enter radius of orbit(in m):\n");
        altitude = getPositiveInput("Please enter the altitude of satellite value from the surface of orbit(in m):\n");

        //output
        cout << "for given values the satellite has to have a velocity of " << velocitycalculation(radius, altitude, mass) << " m/s in " << periodcalculation(radius, altitude, mass) << " seconds\n";

        //ask user if they want to make another calculation
        cout << "\nnow if you want to make another calculation please enter 'yes' or 'no' to go back to menu\n";
        cin >> ans;
        for (auto& c : ans) c = tolower(c);
        while (ans != "yes" && ans != "no") {
            cout << "please enter 'yes' or 'no'\n";
            std::cin >> ans;
            for (auto& c : ans) c = tolower(c);
        }

        system("cls");

    } while (ans == "yes");

    menu();

}

//interactive main menu
void menu() {
    int ans;

    cout << "This is an application of physical representations combined together\n";
    cout << "\nPlease select an option among the following alternatives\n";
    cout << "\n1-Projectile Motion Problem";
    cout << "\n2-Collision of Asteroids";
    cout << "\n3-Ballistic Pendulum";
    cout << "\n4-Satellite Motion";
    cout << "\n\nEnter anything else to stop the program\n";
    cin >> ans;

    //options
    switch (ans) {
    case 1:
        system("cls");
        motion();
        break;
    case 2:
        system("cls");
        collision();
        break;
    case 3:
        system("cls");
        pendulum();
        break;
    case 4:
        system("cls");
        satellite();
        break;
    default:
        system("cls");
        cout << "program has ended thank you for using";
        break;
    }

}

int main() {

    menu();

    return 0;
}