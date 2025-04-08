final NavigationMethod method = new WaypointMethod(); //CartesianMethod, PolarMethod, MouseMethod
final int N_TARGETS = 10;

final ArrayList<Target> targets = new ArrayList<Target>();
long startTime;
PImage colorMap;
PImage maskMap;
final color OBSTACLE_COLOR = color(255, 0, 0);
boolean obstacleAt(float x, float y) {
  final int ix = (int) constrain(x, 0, maskMap.width - 1);
  final int iy = (int) constrain(y, 0, maskMap.height - 1);
  return maskMap.pixels[ix + iy*maskMap.width] == OBSTACLE_COLOR;
}


void settings() {
  colorMap = loadImage("./colorMap.jpg");  
  maskMap = loadImage("./collisionMap.png");
  assert(colorMap.width == maskMap.width && colorMap.height == maskMap.height);
  final int maxWidth = displayWidth - 20;
  if ( colorMap.width > maxWidth) { //TODO all the sizes and speeds should also be rescaled
    colorMap.resize(maxWidth, 0);
    maskMap.resize(maxWidth, 0);
  }
  size( colorMap.width, colorMap.height);
}

void setup() {
  restartTrial( false );
}

HashMap<Integer, Boolean> keys = new HashMap<Integer, Boolean>();
void keyPressed() { 
  keys.put( keyCode, true );
}
void keyReleased() { 
  keys.remove( keyCode );
}
boolean isPressed(int code) { 
  return keys.containsKey( code );
}

void restartTrial(boolean showResult) {
  if (showResult) {
    final long tct = millis() - startTime;
    println("TCT " + tct);
  }
  method.circle.x = width/2;
  method.circle.y = height/2;
  method.clear();
  createCircles(N_TARGETS);
  startTime = millis();
}

ArrayList<Target> toRemove = new ArrayList<Target>();
void draw() {
  //graphics
  image(colorMap, 0, 0);
  for (Target t : targets)
    t.draw();
  method.draw();

  //logic
  toRemove.clear();
  for (Target t : targets) {
    t.logic();
    if ( t.collideWith( method.circle ))
      toRemove.add( t );
  }

  final float prevX = method.circle.x;
  final float prevY = method.circle.y;

  method.logic(); 

  if ( obstacleAt( method.circle.x, method.circle.y) ) {
    method.circle.x = prevX;
    method.circle.y = prevY;
  }
  method.circle.x = constrain(method.circle.x, 0, width);
  method.circle.y = constrain(method.circle.y, 0, height);

  if (toRemove.isEmpty() == false) {
    targets.removeAll( toRemove );
    toRemove.clear();

    if ( targets.isEmpty() ) {
      restartTrial( true );
    }
  }
}


class Circle {
  float x, y, rad = 15;
  color col = color(255, 0, 0);

  public Circle( float x, float y){
    this.x = x;
    this.y = y;
  }
  void draw() {
    fill(col);
    circle(x, y, rad*2);
  }

  boolean collideWith( Circle c) {
    return dist(x, y, c.x, c.y) < rad+c.rad;
  }
  
  void goTowards(float px, float py, float speed){
    float dx = px - x;
    float dy = py - y;
    float dist = sqrt(dx*dx + dy*dy);
    x += dx / dist * speed;
    y += dy / dist * speed;
  }
}

void createCircles(final int n) {
  for (int created = 0; created < n; ) {
    final float x = random(0, width);
    final float y = random(0, height);
    if ( obstacleAt(x, y) == false ) {
      targets.add( new Target(x, y) );
      created += 1;
    }
  }
}


class Target extends Circle {
  float t = 0;
  Target(float x, float y) {
    super(x,y);
    t = random(0, 2*PI);
  }

  void logic() {
    t += 0.1;
  }
  void draw() {
    fill( col );
    circle(x, y+cos(t)*5, rad*2);
  }
}



class NavigationMethod {
  Circle circle;

  NavigationMethod() {
    circle = new Circle(0,0);
    circle.rad = 20;
    circle.col = color(0, 255, 0);
  }

  void logic() {
  }
  void draw() { 
    circle.draw();
  }
  
  void clear(){
  }
}

/* Cartessian Method */
final float MOVE_SPEED = 2;
class CartesianMethod extends NavigationMethod {  
  void logic() {
    if (isPressed( UP )) circle.y -= MOVE_SPEED;
    if (isPressed( DOWN )) circle.y += MOVE_SPEED;
    if (isPressed( LEFT )) circle.x -= MOVE_SPEED;
    if (isPressed( RIGHT )) circle.x += MOVE_SPEED;
  }
}

/* Polar Method */
final float ANGLE_SPEED = radians(5);
class PolarMethod extends NavigationMethod {
  float angle = 0;
  void logic() {
    if (isPressed( UP ) || isPressed( DOWN )) {
      final float dir = isPressed( UP ) ? 1 : -0.8;
      goForward( dir * MOVE_SPEED );
    }
    if (isPressed( LEFT )) angle -= ANGLE_SPEED;
    if (isPressed( RIGHT )) angle += ANGLE_SPEED;
  }

  void goForward( float distance ) {
    final float dx = distance * cos( angle );
    final float dy = distance * sin( angle );
    circle.x += dx;
    circle.y += dy;
  }

  void draw() {
    super.draw();
    final float dx = circle.rad * cos( angle );
    final float dy = circle.rad * sin( angle );
    circle(circle.x + dx, circle.y + dy, 5);
  }
}


/* Mouse Method */
class MouseMethod extends PolarMethod {  
  void logic() {
    angle = atan2( mouseY - circle.y, mouseX - circle.x);
    goForward( MOVE_SPEED );
  }
}


/* Grapple Method */
class GrappleMethod extends CartesianMethod {
}

class WaypointMethod extends NavigationMethod {
  final int SPEED = 6;
  ArrayList<Circle> waypoints = new ArrayList<>();
  boolean prevPressed = false;
  
  void logic(){
    if (prevPressed == false && mousePressed == true){
      waypoints.add(new Circle(mouseX, mouseY));
    }
    prevPressed = mousePressed;
    
    if (!waypoints.isEmpty()) { //<>//
      Circle next_wp = waypoints.get(0);
      circle.goTowards(next_wp.x, next_wp.y, SPEED);
      if (dist(circle.x, circle.y, next_wp.x, next_wp.y) < SPEED)
        waypoints.remove(next_wp);
    }
    
    if (keyPressed && key == 'n')
      waypoints.clear();
  }
  
  void draw(){
    super.draw();
    if (!waypoints.isEmpty()) {
      Circle first_waypoint = waypoints.get(0);
      stroke(#0000FF);
      line(circle.x, circle.y, first_waypoint.x, first_waypoint.y);
      fill(#0000FF);
      circle(first_waypoint.x, first_waypoint.y, first_waypoint.rad);
      
       for(int i = 0; i < waypoints.size()-1; i++) {
         Circle current_wp = waypoints.get(i);
         Circle next_wp = waypoints.get(i+1);
  
         circle(current_wp.x, current_wp.y, current_wp.rad);
         circle(next_wp.x, next_wp.y, next_wp.rad);
         line(current_wp.x, current_wp.y, next_wp.x, next_wp.y);
       }
    }
  }
  
  void clear() {
    waypoints.clear();
  }
  
}
