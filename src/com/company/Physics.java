package com.company;

import java.util.ArrayList;

public class Physics {

    // VARIABLES
    // GRAVITY
    private double gravity = 10;
    // AIR
    private double airDensity = 1;
    private double dragCoefficient = 0.47;  // for the sphere is 0.47
    private double airResistance;
    private double windspeed; //   m/s
    private double windspeedx = 0;
    private double windspeedy;
    // ANGLE
    private double angle;
    private double startAngle;
    private double groundAngle;
    // AREA
    private double area = 1;
    // SPEED
    private double initialSpeed;
    private double speed;
    private double currentHorizontalSpeed;
    private double currentVerticalSpeed;
    private double currentAirSpeed;
    // MASS
    private double mass = 1;
    //POSITION
    private double initialHeight;
    private ArrayList<Double> idealHeight = new ArrayList<Double>(0);
    private ArrayList<Double> idealDistance = new ArrayList<Double>(0);
    private ArrayList<Double> height = new ArrayList<Double>(0);
    private ArrayList<Double> distance = new ArrayList<Double>(0);
    //VECTORS
    private ArrayList<Double> idealX = new ArrayList<Double>(0);
    private ArrayList<Double> idealY = new ArrayList<Double>(0);
    private ArrayList<Double> idealFW = new ArrayList<Double>(0);
    private ArrayList<Double> vectorX = new ArrayList<Double>(0);
    private ArrayList<Double> vectorY = new ArrayList<Double>(0);
    private ArrayList<Double> vectorFW = new ArrayList<Double>(0);
    // TIME
    private double time = 0;
    private double idealFlightTime;
    private double flighTime;
    private int timeInterval = 0;
    public static final double PHYSICS_REFRESH_RATE = .1;
    // ANOTHER
    private double k;
    private double lamda; //I have no idea how to name that
    private double timeUp;
    private double xx;

    //FOR GUI
    public void setGravity(double gravity) {
        this.gravity = gravity;
    }

    public void setAirdensity(double airdensity) { //gęstość powietrza
        this.airDensity = airdensity;
    }

    public void setDragCoefficient(double dragCoefficient) {  //współczynnik oporu obiektu  Przypadku kuli jest to 0.47
        this.dragCoefficient = dragCoefficient;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void setArea(double area) { //przekrój obiektu
        this.area = area;
    }

    public void setInitialspeed(double initialSpeed) {
        this.initialSpeed = initialSpeed;
    }

    public void setMass(double mass) {
        this.mass = mass;
    }

    // CALCULATIONS
    private double calculateAirResistance() //without V
    {
        k = ((dragCoefficient * airDensity * area) / (2 * mass));
        return k;
    }

    private double calculateTW() {
        timeUp = (Math.atan(lamda) / Math.sqrt(k * gravity));
        return timeUp;
    }

    private double calcLambda() {
        lamda = Math.atan(initialSpeed * StrictMath.sin(angle) * Math.sqrt(k / gravity));
        return lamda;
    }

    private double calcXX() {
        xx = ((initialSpeed * StrictMath.sin(angle)) + StrictMath.sqrt(gravity / k)) /
                ((initialSpeed * StrictMath.sin(angle)) - StrictMath.sqrt(gravity / k));
        return xx;
    }

    private void calculateIdealFlightTime() {
        idealFlightTime = ((2 * initialSpeed * StrictMath.sin(angle)) / gravity);
        System.out.println("Flight time: " + idealFlightTime);
    }

    private void calculateFlightTime() {
    }  //Numerical Methods needed

    public void calculateCurrentIdealVectors() {
        timeInterval = 0;
        time = 0;
        timeInterval++;
        do {
            time += PHYSICS_REFRESH_RATE; // = 0 + 0,1 itd
            idealX.add(currentHorizontalSpeed);
            idealY.add(currentHorizontalSpeed - (gravity * time));
            idealFW.add(StrictMath.hypot(currentHorizontalSpeed, currentVerticalSpeed));
            timeInterval++;
        } while (idealHeight.get(timeInterval - 1) > 0);
    }

    public void calculateCurrentVectors() {
        timeInterval = 0;
        time = 0;
        timeInterval++;
        do {
            time += PHYSICS_REFRESH_RATE; // = 0 + 0,1 itd
            calculateAirResistance(); // 04.08 wrong output now i use K without V
            angle = Math.atan2(currentVerticalSpeed, currentHorizontalSpeed);
            vectorX.add(currentHorizontalSpeed = (currentHorizontalSpeed - ((airResistance * StrictMath.cos(angle)) / mass) * time));
            vectorY.add(currentVerticalSpeed -= (gravity + ((airResistance * StrictMath.sin(angle)) / mass)) * time);
            currentAirSpeed = StrictMath.hypot(currentHorizontalSpeed, currentVerticalSpeed);
            vectorFW.add(StrictMath.hypot(currentHorizontalSpeed, currentVerticalSpeed));
            timeInterval++;
        } while (idealHeight.get(timeInterval - 1) > 0);
        distance.add(0.0);
    }

    public Physics(double initialspeed, double angle, double initialHeight, double groundAngle) {
        this.angle = StrictMath.toRadians(angle);
        this.initialSpeed = initialspeed;
        this.initialHeight = initialHeight;
        this.groundAngle = groundAngle;
        distance.add(0.0);
        speed = initialSpeed;
        currentHorizontalSpeed = StrictMath.cos(this.angle) * initialspeed;
        currentVerticalSpeed = StrictMath.sin(this.angle) * initialspeed;
        currentAirSpeed = initialspeed;
        startAngle = Math.atan2(currentVerticalSpeed, currentHorizontalSpeed);
        height.add(initialHeight);
        distance.add(0.0);
    }

    public void runSimulation() {

        idealHeight.add(initialHeight);
        idealDistance.add(0.0);
        height.add(initialHeight);
        distance.add(0.0);
        calculateIdealFlightTime();
        calculateAirResistance();
        writeInitiaVariables();

        //---------------------------------     Motion without airresistance and wind
        timeInterval++;
        time = 0;
        do {
            time += PHYSICS_REFRESH_RATE; // = 0 + 0,1...
            idealHeight.add((initialSpeed * time * StrictMath.sin(angle) - ((gravity / 2) * Math.pow(time, 2))));
            idealDistance.add(initialSpeed * time * StrictMath.cos(angle));
            timeInterval++;
        }
        while (idealHeight.get(timeInterval - 1) > 0);
       // writeIdealTrajectory();
       // calculateCurrentIdealVectors();
       // writeIdealVectors();



        //---------------------------------     Motion with...

        double xspeed;//speed x-axis
        //must check currentverticalspeed after ideal calculations
        calcLambda();
        calcXX();
        calculateTW();
        System.out.println(lamda + " lambda");
        System.out.println(timeUp + " timpeup");
        xspeed = windspeedx - currentVerticalSpeed;

        timeInterval = 1;
        time = 0;
        do {
            time += PHYSICS_REFRESH_RATE;
            //something wrong height is < 0
            if (time < timeUp) {
                height.add(initialHeight + ((1 / k) * ((StrictMath.log(StrictMath.sin(Math.sqrt(k * gravity) * time + lamda))) / StrictMath.sin(lamda)))); //wznoszenie
            }
            if (time > timeUp) {
                height.add(initialHeight + ((1 / k) * (Math.sqrt(k * gravity) * (time - timeUp) -
                        StrictMath.log((StrictMath.exp(2 * StrictMath.sqrt(k * gravity) * (time - timeUp)) - xx) / (1 - xx)))));
            }
        } while (height.get(timeInterval - 1) > 0);

        time = 0;
        timeInterval = 1;
        do{
            time += PHYSICS_REFRESH_RATE;
        if (xspeed < 0) {
            distance.add((windspeedx * time) + ((1 / k) * StrictMath.log(1 - k * xspeed * time)));
        } else if (xspeed > 0) {
            distance.add((windspeedx * time) - ((1 / k) * StrictMath.log(1 + k * xspeed * time)));
        } else if (xspeed == 0) {
            //check this, check variables
            distance.add(initialSpeed * time * StrictMath.cos(angle));
        }
        timeInterval++;
        }while (height.get(timeInterval - 1) > 0);
        writeTrajectory();
        calculateCurrentVectors();
        //writeVectors();


    }

    //TESTS
    public void writeInitiaVariables() {
        System.out.println("Initial horizontal speed: " + currentHorizontalSpeed + " Initial vertical speed: " + currentVerticalSpeed + " Initial air speed: "
                + currentAirSpeed + " Speed: " + speed + " Angle: " + angle + " Initial ground angle: " + groundAngle + "\n");
    }

    public void writeIdealTrajectory() {
        for (int i = 0; i < idealHeight.size() - 1; i++) {
            Double wysokosc = (Double) idealHeight.get(i);
            System.out.println("IdealHeight: " + i + ":" + wysokosc);
        }
        for (int i = 0; i < idealDistance.size() - 1; i++) {
            Double dystans = (Double) idealDistance.get(i);
            System.out.println("IdealDistance: " + i + ":" + dystans);
        }
    }

    public void writeIdealVectors() {
        for (int i = 0; i < idealX.size() - 1; i++) {
            Double wektorx = (Double) idealX.get(i);
            System.out.println("Vector X: " + i + ":" + wektorx);
        }
        for (int i = 0; i < idealY.size() - 1; i++) {
            Double wektory = (Double) idealY.get(i);
            System.out.println("Vector Y:  " + i + ":" + wektory);
        }
        for (int i = 0; i < idealFW.size() - 1; i++) {
            Double wektorfw = (Double) idealFW.get(i);
            System.out.println("Vector FW:  " + i + ":" + wektorfw);
        }
    }

    public void writeTrajectory() {
        for (int i = 0; i < height.size() - 1; i++) {
            Double wysokoscz = (Double) height.get(i);
            System.out.println("Height: " + i + ":" + wysokoscz);
        }
        for (int i = 0; i < distance.size() - 1; i++) {
            Double dystansz = (Double) distance.get(i);
            System.out.println("Distance: " + i + ":" + dystansz);
        }
    }

    public void writeVectors() {
        for (int i = 0; i < vectorX.size() - 1; i++) {
            Double wektorx = (Double) vectorX.get(i);
            System.out.println("Vector X: " + i + ":" + wektorx);
        }
        for (int i = 0; i < vectorY.size() - 1; i++) {
            Double wektory = (Double) vectorY.get(i);
            System.out.println("Vector Y: " + i + ":" + wektory);
        }
        for (int i = 0; i < vectorFW.size() - 1; i++) {
            Double wektorfw = (Double) vectorFW.get(i);
            System.out.println("Vector FW: " + i + ":" + wektorfw);
        }
    }
}
