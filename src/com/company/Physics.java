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
    private double windspeedx;
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
    private double idealTime = 0;
    private double idealFlightTime;
    private double flighTime;
    private int timeInterval = 0;
    public static final double PHYSICS_REFRESH_RATE = .1;
    // ANOTHER
    private double k;
    private double lamda; //I have no idea how to name that

    //FOR GUI
    public void setGravity(double gravity){
        this.gravity = gravity;
    }
    public void setAirdensity(double airdensity){ //gęstość powietrza
        this.airDensity = airdensity;
    }
    public void setDragCoefficient(double dragCoefficient){  //współczynnik oporu obiektu  Przypadku kuli jest to 0.47
        this.dragCoefficient = dragCoefficient;
    }
    public void setAngle(double angle){
        this.angle = angle;
    }
    public void setArea(double area){ //przekrój obiektu
        this.area = area;
    }
    public void setInitialspeed(double initialSpeed){
        this.initialSpeed = initialSpeed;
    }
    public void setMass(double mass){
        this.mass = mass;
    }

    // CALCULATIONS
    private double calcK()
    {
     k = ((airResistance * airDensity * dragCoefficient) / (2 * mass));
        return  k;
    }
    private void  calcLambda() {
        lamda = Math.atan(initialSpeed * StrictMath.sin(angle) * Math.sqrt(k/gravity));
    }

    private void calculateAirResistance()
    {
        airResistance = dragCoefficient * area * ((airDensity * StrictMath.pow(currentAirSpeed,2))/2);
        System.out.println("Air resistance is: " + airResistance);
    }

    private void calculateIdealFlightTime()
    {
        idealFlightTime = ((2* initialSpeed * StrictMath.sin(angle))/gravity);
        System.out.println("Flight time: " + idealFlightTime);
    }

    private void calculateFlightTime(){}  //Numerical Methods needed

    public void calculateCurrentIdealVectors() {
        timeInterval = 0;
        idealTime = 0;
        timeInterval++;
        do {
            idealTime += PHYSICS_REFRESH_RATE; // = 0 + 0,1 itd
            idealX.add(currentHorizontalSpeed);
            idealY.add(currentHorizontalSpeed - (gravity * idealTime));
            idealFW.add(StrictMath.hypot(currentHorizontalSpeed, currentVerticalSpeed));
            timeInterval++;
        } while (idealHeight.get(timeInterval - 1) > 0);
    }

    public void calculateCurrentVectors(){
        timeInterval = 0;
        idealTime = 0;
        timeInterval++;
        do
            {
            idealTime += PHYSICS_REFRESH_RATE; // = 0 + 0,1 itd
            calculateAirResistance(); // wrong output
            angle = Math.atan2(currentVerticalSpeed, currentHorizontalSpeed);
            vectorX.add(currentHorizontalSpeed = (currentHorizontalSpeed - ((airResistance * StrictMath.cos(angle)) / mass) * idealTime));
            vectorY.add(currentVerticalSpeed -= (gravity + ((airResistance * StrictMath.sin(angle)) / mass)) * idealTime);
            currentAirSpeed = StrictMath.hypot(currentHorizontalSpeed,currentVerticalSpeed);
            vectorFW.add(StrictMath.hypot(currentHorizontalSpeed, currentVerticalSpeed));
            timeInterval++;
            } while(idealHeight.get(timeInterval - 1) > 0);
        distance.add(0.0);
    }

    public Physics (double initialspeed, double angle, double initialHeight, double groundAngle)
    {
        this.angle = StrictMath.toRadians(angle);
        this.initialSpeed = initialspeed;
        this.initialHeight = initialHeight;
        this.groundAngle = groundAngle;distance.add(0.0);
        speed = initialSpeed;
        currentHorizontalSpeed = StrictMath.cos(this.angle) * initialspeed;
        currentVerticalSpeed = StrictMath.sin(this.angle) * initialspeed;
        currentAirSpeed = initialspeed;
        startAngle = Math.atan2(currentVerticalSpeed, currentHorizontalSpeed);
        height.add(initialHeight);
        distance.add(0.0);
    }

    public void runSimulation(){
        idealHeight.add(initialHeight);
        idealDistance.add(0.0);
        calculateIdealFlightTime();
        // currentAirSpeed = StrictMath.hypot(currentHorizontalSpeed,currentVerticalSpeed);  // must check it
         calculateAirResistance();
         writeInitiaVariables();
         timeInterval++;
        do{
            idealTime += PHYSICS_REFRESH_RATE; // = 0 + 0,1...
            idealHeight.add((initialSpeed * idealTime * StrictMath.sin(angle) - ((gravity/2)* Math.pow(idealTime,2))));
            idealDistance.add(initialSpeed * idealTime * StrictMath.cos(angle));
            timeInterval++;
        }
        while(idealHeight.get(timeInterval - 1) > 0);
       // writeIdealTrajectory();
       // calculateCurrentIdealVectors();
       // writeIdealVectors();


        timeInterval = 1;
        private double windinfluence;
        //must check currentverticalspeed after ideal calculations
        windinfluence = windspeed - currentVerticalSpeed;
        calcK();

        if(windinfluence < 0){
            do {
                initialSpeed = 1;
                idealTime += PHYSICS_REFRESH_RATE;
                //old
               // height.add((((((mass*initialSpeed)/airResistance))*StrictMath.sin(startAngle)+((StrictMath.pow(mass,2)*gravity)/StrictMath.pow(airResistance,2)))*
                //        (1-StrictMath.exp(-(airResistance/mass)*idealTime)))-(((mass * gravity)/airResistance)*idealTime));

                //check variables and result
                height.add(initialHeight + ((1/k) * (Math.log(Math.sin(Math.sqrt(k * gravity) * time + lamda)))/StrictMath.sin(lamda)));
                //check variables and result
                distance.add(windspeedx * time + (1/k) * Math.log(1-(k*windinfluence * time)));


                //old distance  distance.add(((mass*initialSpeed)/airResistance)*StrictMath.cos(startAngle)*(1-StrictMath.exp(-(airResistance/mass)*idealTime)));
                timeInterval++;
            } while(idealHeight.get(timeInterval - 1) > 0);
            angle = Math.atan2(currentVerticalSpeed, currentHorizontalSpeed);
            //  writeTrajectory();
            // calculateCurrentVectors();
            // writeVectors();
        }
        else if(windinfluence > 0){

        }
        else if(windinfluence = 0){
            //check this, check variables
            distance.add(initialSpeed * time * StrictMath.cos(angle);
        }


    }

    //TESTS
    public void writeInitiaVariables(){
        System.out.println("Initial horizontal speed: "+ currentHorizontalSpeed + " Initial vertical speed: " + currentVerticalSpeed + " Initial air speed: "
                + currentAirSpeed + " Speed: " + speed + " Angle: " + angle + " Initial ground angle: " + groundAngle + "\n");
        }

    public void writeIdealTrajectory(){
        for(int i =0; i < idealHeight.size() - 1; i++){
            Double wysokosc = (Double)idealHeight.get(i);
            System.out.println("IdealHeight: " + i + ":" + wysokosc);
        }
        for(int i = 0; i < idealDistance.size() - 1; i++){
            Double dystans = (Double)idealDistance.get(i);
            System.out.println("IdealDistance: " + i + ":" + dystans);
        }
    }

    public void writeIdealVectors(){
        for(int i = 0; i < idealX.size() - 1; i++){
            Double wektorx = (Double)idealX.get(i);
            System.out.println("Vector X: " + i + ":" + wektorx);
        }
        for(int i = 0; i < idealY.size() - 1; i++){
            Double wektory = (Double)idealY.get(i);
            System.out.println("Vector Y:  " + i + ":" + wektory);
        }
        for(int i = 0; i < idealFW.size() - 1; i++){
            Double wektorfw = (Double)idealFW.get(i);
            System.out.println("Vector FW:  " + i + ":" + wektorfw);
        }
    }

    public void writeTrajectory(){
        for(int i =0; i < height.size() - 1; i++){
            Double wysokoscz = (Double)height.get(i);
            System.out.println("Height: " + i + ":" + wysokoscz);
        }
        for(int i = 0; i < distance.size() - 1; i++){
            Double dystansz = (Double)distance.get(i);
            System.out.println("Distance: " + i + ":" + dystansz);
        }
    }

    public void writeVectors(){
        for(int i = 0; i < vectorX.size() - 1; i++){
            Double wektorx = (Double)vectorX.get(i);
            System.out.println("Vector X: " + i + ":" + wektorx);
        }
        for(int i = 0; i < vectorY.size() - 1; i++){
            Double wektory = (Double)vectorY.get(i);
            System.out.println("Vector Y: " + i + ":" + wektory);
        }
        for(int i = 0; i < vectorFW.size() - 1; i++){
            Double wektorfw = (Double)vectorFW.get(i);
            System.out.println("Vector FW: " + i + ":" + wektorfw);
        }
    }
}
