/* view.cpp
 
 Laura Toma, Ivy Xing, Zackery Leman
 
 What it does: See readme.
 
 */

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <queue>
#include <vector>
#include "geom.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;


const int ROBOT_SIZE = 20;
const int WINDOWSIZE = 500;
const int NUM_SEGMENTS = 800; // Number of segments that form the circle.

GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat orange[3] = {1.0, 0.5, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};

GLint fillmode = 0;


/**** Forward declarations of functions ****/

/* Display and responsive functions */
void display(void);
void keypress(unsigned char key, int x, int y);
void mouse(int button, int state, int Mx, int My);
void startMoving(void);


/* Drawing functions */
void drawCircle(float cx, float cy, float r, int num_segments);
void drawCirclesAroundVertices(void);
void drawSegment(segment2D s);
void drawPolygon(void);
void drawPolygons(void);
void drawPolygonInProgress(void);


/* Helper computation functions */
int intersectPolygon(segment2D current);
int insidePolygon(point2D p);
void drawRobot(double robotCenterX, double robotCenterY);
bool hasBeenToSate(double x, double y);
bool atGoal(double robotCenterX, double robotCenterY);
bool outOfBounds(point2D pointToCheck);
bool robotHitsObstacle(double robotCenterX, double robotCenterY);
State* getState(double robotCenterX, double  robotCenterY,State robotState);
State* getDiagState(double robotCenterX, double  robotCenterY,State robotState);
State* generateSuccessors(State currentRobotState);
vector<point2D>  findShortestPath();
vector<point2D>  findShortestPath();
void interestingTestCase();


/**** Global variables ****/
int hasFinishedPolygon = 0; // Whether the user has finished drawing the current polygon.
bool hasFinishedPolygons = false; // Whether the user has finished drawing the polygons.
point2D clickPoint; // The point of a click
point2D robotStartPos = {0, 0};
point2D robotEndPos = {0, 0};
point2D robotPosition;
float  robotAngle;
bool startHasBeenSet = false; //Have selected robot start point
bool moveForward = true; //direction to move robot
int constant = ROBOT_SIZE;
//Tracks what states have already been visited
std::map<std::string, std::string> exploredMap;

vector<vector<segment2D> >  polygons;
// The array of segments that form the polygon currently being drawn.
vector<segment2D>  polygonSegments;
// The vertices of the current polygon being drawn.
vector<point2D> polygonVertices;

//The Shortest path that the robot will travel on
vector<point2D> shortestpath;

//The Shortest path that the robot will travel on
vector<double> shortestpathAngle;

//Priority queue comparison function that uses the cost incurred by traveling on the current path and the heuristic euclidean distance from the position to the goal state.
class CompareStateDistance {
public:
    bool operator()(State& t1, State& t2)
    {
        if (t1.distance + t1.cost > t2.distance + t2.cost ) return true;
        return false;
    }
};

//Priority queue for the frontier states
priority_queue<State,vector<State>,CompareStateDistance> nextStateQueue;


//Check if any of the four sides of the rectangular (square) robot intersects any of the edges of the obstacles
void drawRobot(double robotCenterX, double robotCenterY,float angle){
    
    //Generate four corners
    point2D bottomRight = {0.0 + constant, 0.0 - constant};
    double tempX = ( bottomRight.x * cos(angle) -  bottomRight.y * sin(angle) )+ robotCenterX;
    double  tempY = (bottomRight.x * sin(angle) +  bottomRight.y * cos(angle)) + robotCenterY;
    
    bottomRight.x = tempX;
    bottomRight.y = tempY;
    
    point2D bottomLeft =  {0.0  - constant, 0.0 - constant};
    tempX = ( bottomLeft.x * cos(angle) -  bottomLeft.y * sin(angle) )+ robotCenterX;
    tempY = ( bottomLeft.x * sin(angle) +  bottomLeft.y * cos(angle) ) + robotCenterY;
    
    bottomLeft.x = tempX;
    bottomLeft.y = tempY;
    
    point2D topLeft =  {0.0 - constant, 0.0 + constant};
    tempX = ( topLeft.x * cos(angle) -  topLeft.y * sin(angle) )+ robotCenterX;
    tempY = (topLeft.x * sin(angle) +  topLeft.y * cos(angle) ) + robotCenterY;
    
    topLeft.x = tempX;
    topLeft.y = tempY;
    
    point2D topRight =  {0.0 + constant, 0.0 + constant};
    tempX = ( topRight.x * cos(angle) -  topRight.y * sin(angle) )+ robotCenterX;
    tempY =( topRight.x * sin(angle) +  topRight.y * cos(angle) ) + robotCenterY;
    
    topRight.x = tempX;
    topRight.y = tempY;
    
    
    segment2D leftVert = {topLeft,bottomLeft};
    segment2D rightVert =  {topRight,bottomRight};
    segment2D top =  {topLeft,topRight};
    segment2D bottom =  {bottomLeft,bottomRight};
    
    glColor3fv(cyan);
    drawSegment(leftVert);
    drawSegment(rightVert);
    drawSegment(top);
    drawSegment(bottom);
    
}

/*Method to prevent locations from being revisited. Done in constant time with hashmap*/
bool hasBeenToSate(double x, double y) {
    
    std::ostringstream oss;
    oss << floor(x) << " " << floor(y);
    std::string tempString = oss.str();
    
    // check if key is present
    if (exploredMap.find(tempString) != exploredMap.end()){
        //Map Already Contains Point
        return true;
    } else{
        exploredMap[tempString] = "";
        return false;
    }
}

//Returns whether at goal state
bool atGoal(double robotCenterX, double robotCenterY){
    
    if (doubleEqual(robotCenterX,robotEndPos.x) && doubleEqual(robotCenterY,robotEndPos.y)) {
        return true;
    }
    return false;
}

//Check if a point is outside of the window
bool outOfBounds(point2D pointToCheck){
    
    if (pointToCheck.x < 0 || pointToCheck.x > WINDOWSIZE || pointToCheck.y < 0 || pointToCheck.y > WINDOWSIZE ) {
        return true;
    }
    return false;
}


//Check if any of the four sides of the rectangular (square) robot intersects any of the edges of the obstacles
bool robotHitsObstacle(double robotCenterX, double robotCenterY){
    
    //Generate four corners
    point2D bottomRight = {robotCenterX + constant, robotCenterY - constant};
    point2D bottomLeft =  {robotCenterX - constant, robotCenterY - constant};
    point2D topLeft =  {robotCenterX - constant, robotCenterY + constant};
    point2D topRight =  {robotCenterX + constant, robotCenterY + constant};
    
    //First check if any of the points are out of bounds
    if (outOfBounds(bottomRight) || outOfBounds(bottomLeft) || outOfBounds(topLeft) || outOfBounds(topRight)){
        return true;
    }
    
    //Generate for sides from corners
    segment2D leftVert = {topLeft,bottomLeft};
    segment2D rightVert =  {topRight,bottomRight};
    segment2D top =  {topLeft,topRight};
    segment2D bottom =  {bottomLeft,bottomRight};
    
    //Test for intersections
    for (int i = 0; i < polygons.size(); i++) {
        
        vector<segment2D> polygonSegments = polygons.at(i);
        
        for (int j = 0; j < polygonSegments.size(); j++) {
            if (intersect(leftVert, polygonSegments.at(j)) ||  intersect(rightVert, polygonSegments.at(j))  ||  intersect(top, polygonSegments.at(j))  || intersect(bottom, polygonSegments.at(j)) ){
                return true;
            }
            
        }
        
        
    }
    
    
    return false;
}


//Check if any of the four sides of the rectangular (square) robot intersects any of the edges of the obstacles
bool robotHitsObstacleWithRotation(double robotCenterX, double robotCenterY,float angle){
    
    //Generate four corners
    point2D bottomRight = {0.0 + constant, 0.0 - constant};
    double tempX = ( bottomRight.x * cos(angle) -  bottomRight.y * sin(angle) )+ robotCenterX;
    double  tempY = (bottomRight.x * sin(angle) +  bottomRight.y * cos(angle)) + robotCenterY;
    
    bottomRight.x = tempX;
    bottomRight.y = tempY;
    
    point2D bottomLeft =  {0.0  - constant, 0.0 - constant};
    tempX = ( bottomLeft.x * cos(angle) -  bottomLeft.y * sin(angle) )+ robotCenterX;
    tempY =( bottomLeft.x * sin(angle) +  bottomLeft.y * cos(angle) ) + robotCenterY;
    
    bottomLeft.x = tempX;
    bottomLeft.y = tempY;
    
    point2D topLeft =  {0.0 - constant, 0.0 + constant};
    tempX = ( topLeft.x * cos(angle) -  topLeft.y * sin(angle) )+ robotCenterX;
    tempY = (topLeft.x * sin(angle) +  topLeft.y * cos(angle) ) + robotCenterY;
    
    topLeft.x = tempX;
    topLeft.y = tempY;
    
    point2D topRight =  {0.0 + constant, 0.0 + constant};
    tempX = ( topRight.x * cos(angle) -  topRight.y * sin(angle) )+ robotCenterX;
    tempY =( topRight.x * sin(angle) +  topRight.y * cos(angle) ) + robotCenterY;
    
    topRight.x = tempX;
    topRight.y = tempY;
    
    
    //First check if any of the points are out of bounds
    if (outOfBounds(bottomRight) || outOfBounds(bottomLeft) || outOfBounds(topLeft) || outOfBounds(topRight)){
        return true;
    }
    
    //Generate for sides from corners
    segment2D leftVert = {topLeft,bottomLeft};
    segment2D rightVert =  {topRight,bottomRight};
    segment2D top =  {topLeft,topRight};
    segment2D bottom =  {bottomLeft,bottomRight};
    
    //Test for intersections
    for (int i = 0; i < polygons.size(); i++) {
        
        vector<segment2D> polygonSegments = polygons.at(i);
        
        for (int j = 0; j < polygonSegments.size(); j++) {
            if (intersect(leftVert, polygonSegments.at(j)) ||  intersect(rightVert, polygonSegments.at(j))  ||  intersect(top, polygonSegments.at(j))  || intersect(bottom, polygonSegments.at(j)) ){
                return true;
            }
            
        }
        
        
    }
    
    
    return false;
}

//Finds the first angle that does not cause a intersection with an obstacle for that location
double findRotation(double robotCenterX, double robotCenterY){
    
    for (float rotation = 0.0; rotation < 2 * M_PI; rotation += 0.05) {
        if (!robotHitsObstacleWithRotation(robotCenterX,robotCenterY,rotation)){
            return rotation;
        }
    }
    return -1;
}

//Return the state at this new position given past state history
State* getState(double robotCenterX, double  robotCenterY,State robotState){
    
    
    if (!hasBeenToSate(robotCenterX, robotCenterY)) {
        
        if ( !robotHitsObstacleWithRotation(robotCenterX, robotCenterY,robotState.angle)) {
            State * nextPossibleState = new State();
            nextPossibleState->location = {robotCenterX, robotCenterY};
            nextPossibleState->path = robotState.path;
            nextPossibleState->path.push_back(nextPossibleState->location );
            nextPossibleState->distance = distance_(nextPossibleState->location,robotEndPos);
            nextPossibleState->cost = robotState.cost + 1;
            nextPossibleState->angle = robotState.angle;
            nextPossibleState->pathAngle = robotState.pathAngle;
            nextPossibleState-> pathAngle.push_back(nextPossibleState->angle);
            
            
            nextStateQueue.push(*nextPossibleState);
            if (atGoal(robotCenterX, robotCenterY)) {
                return nextPossibleState;
            }
            
        }else{
            double angle = findRotation(robotCenterX, robotCenterY);
            if (doubleEqual(-1.0, angle)){
                return NULL;
            }
            
            State * nextPossibleState = new State();
            nextPossibleState->location = {robotCenterX, robotCenterY};
            nextPossibleState->path = robotState.path;
            nextPossibleState->path.push_back(nextPossibleState->location );
            nextPossibleState->distance = distance_(nextPossibleState->location,robotEndPos);
            nextPossibleState->cost = robotState.cost + 1;
            nextPossibleState->angle = angle;
            nextPossibleState->pathAngle = robotState.pathAngle;
            nextPossibleState-> pathAngle.push_back(nextPossibleState->angle);
            
            
            nextStateQueue.push(*nextPossibleState);
            if (atGoal(robotCenterX, robotCenterY)) {
                return nextPossibleState;
            }
            
        }
    }
    return NULL;
}



//Adds all viable sucessor states to current state to the priority queue
State* generateSuccessors(State currentRobotState){
    
    point2D robotCenter = currentRobotState.location;
    
    //Up
    State * state = getState(robotCenter.x, robotCenter.y + 1,currentRobotState);
    
    if ( state != NULL) {
        return state;
    }
    
    //Down
    state = getState(robotCenter.x, robotCenter.y - 1,currentRobotState);
    
    if ( state != NULL) {
        return state;
    }
    
    //Right
    state = getState(robotCenter.x + 1, robotCenter.y,currentRobotState);
    if ( state != NULL) {
        return state;
    }
    
    //Left
    
    state = getState(robotCenter.x - 1, robotCenter.y,currentRobotState);
    if ( state != NULL) {
        return state;
    }
    
    return NULL;
    
}

//Called to find the shortest path around the obstacles. Returns this path.
vector<point2D>  findShortestPath(){
    
    State * nextPossibleState = new State();
    nextPossibleState->location = robotStartPos;
    nextPossibleState->distance = distance_(nextPossibleState->location,robotEndPos);
    generateSuccessors(*nextPossibleState);
    hasBeenToSate(robotStartPos.x, robotStartPos.y);
    
    
    State currentState;
    while (!nextStateQueue.empty()) {
        
        currentState = nextStateQueue.top();
        nextStateQueue.pop();
        State *terminal = generateSuccessors(currentState);
        if (terminal != NULL) {
            shortestpathAngle = terminal->pathAngle;
            //Returns the shortest path
            return terminal->path;
        }
        
    }
    //Returns if there is no solution
    vector<point2D>  empty;
    return empty;
    
}



/* ****************************** */
int main(int argc, char** argv) {
    
    
    /* initialize GLUT  */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
    glutInitWindowPosition(100,100);
    glutCreateWindow(argv[0]);
    
    /* register callback functions */
    glutDisplayFunc(display);
    glutKeyboardFunc(keypress);
    glutMouseFunc(mouse);
    
    /* init GL */
    /* set background color black*/
    glClearColor(0.3,0.3, 0.3, 0.3);
    /* here we can enable depth testing and double buffering and so
     on */
    
    /* give control to event handler */
    glutMainLoop();
    
    return 0;
}

/**** Drawing functions ****/


// Draw a line segment.
void drawSegment(segment2D s) {
    glBegin(GL_LINES);
    glVertex2f(s.start.x, s.start.y);
    glVertex2f(s.end.x, s.end.y);
    glEnd();
}

// Draw path segment.
void drawSegmentFromPoints(point2D start, point2D end ) {
    glBegin(GL_LINES);
    glVertex2f(start.x, start.y);
    glVertex2f(end.x, end.y);
    glEnd();
}

//Draes path o robot
void drawPath(){
    glColor3fv(cyan);
    for (int i = 1; i < shortestpath.size(); i++) {
        drawSegmentFromPoints(shortestpath.at(i - 1),shortestpath.at(i));
    }
    
}

// Generic openGL draw circle method.
void drawCircle(float cx, float cy, float r, int num_segments, int filled) {
    if (filled) {
        float x2, y2;
        float angle;
        
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(cx, cy);
        for (angle = 1.0f; angle < 361.0f; angle += 0.2) {
            x2 = cx + sin(angle) * r;
            y2 = cy + cos(angle) * r;
            glVertex2f(x2, y2);
        }
        glEnd();
    } else {
        float theta = 2 * 3.1415926 / float(num_segments);
        float c = cosf(theta); //precalculate the sine and cosine
        float s = sinf(theta);
        float t;
        float x = r; //we start at angle = 0
        float y = 0;
        
        glBegin(GL_LINE_LOOP);
        for(int ii = 0; ii < num_segments; ii++) {
            glVertex2f(x + cx, y + cy);//output vertex
            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
        glEnd();
    }
}

// Draw circles around each vertex to make it more visible.
void drawCirclesAroundVertices(void) {
    for (int i = 0; i < polygonVertices.size(); i++) {
        point2D v = polygonVertices[i];
        drawCircle(v.x, v.y, RADIUS, NUM_SEGMENTS, 0);
    }
}

// Draw the shape of the polygon.
void drawPolygon(vector<segment2D> segments) {
    glColor3fv(red);
    for (int i = 0; i < segments.size(); i++) {
        drawSegment(segments[i]);
    }
}

// Draw all polygons.
void drawPolygons(void) {
    for (int i = 0; i < polygons.size(); i++) {
        drawPolygon(polygons[i]);
    }
}

// Draw the shape of the polygon.
void drawPolygonInProgress(void) {
    glColor3fv(red);
    for (int i = 0; i < polygonSegments.size(); i++) {
        drawSegment(polygonSegments[i]);
    }
}

// Draw the start and end points.
void drawStartEnd(void) {
    if (startHasBeenSet) {
        glColor3fv(green);
        drawCircle(robotStartPos.x, robotStartPos.y, RADIUS, NUM_SEGMENTS, 1);
        glColor3fv(red);
        drawCircle(robotEndPos.x, robotEndPos.y, RADIUS, NUM_SEGMENTS, 1);
    }
}


/**** Computation Functions ****/

// Return 1 if current edge intersects with any other edge of the polygon;
// 0 otherwise.
int intersectPolygon(segment2D current) {
    if (polygonSegments.size() > 0) {
        // The last segment is the current segment.
        for (int i = 0; i < polygonSegments.size() - 1; i++) {
            if (intersect(current, polygonSegments[i])) {
                return 1;
            }
        }
    }
    return  0;
}

// Return 1 if the visibility point is inside the polygon; 0 otherwise.
int insidePolygon(point2D p) {
    // Draw lines starting from the visibility point parallel to the x-axis.
    segment2D testLineRight, testLineLeft;
    testLineRight.start = p;
    testLineRight.end = {WINDOWSIZE * 2, p.y };
    testLineLeft.start = p;
    testLineLeft.end  = {-1 * WINDOWSIZE, p.y};
    
    // Count the number of intersections with the polygon.
    int numIntersectionsRight = 0, numIntersectionsLeft = 0;
    for (int i = 0; i < polygonSegments.size(); i++) {
        if (intersect_proper(testLineRight, polygonSegments[i])) {
            numIntersectionsRight++;
        }
        if (intersect_proper(testLineLeft, polygonSegments[i])) {
            numIntersectionsLeft++;
        }
    }
    // An odd number of intersections means the point is insde.
    if (numIntersectionsRight % 2 == 1 ||
        numIntersectionsLeft % 2 == 1) {
        return 1;
    }
    return 0;
}


/**** Responsive and Display Functions ****/

// Respond to mouse clicks.
void mouse(int button, int state, int Mx, int My) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        point2D mouseClick = {(double)Mx, (double)(WINDOWSIZE - My)};
        
        // User finished drawing polygon, so compute visibility area.
        if (hasFinishedPolygons) {
            clickPoint = mouseClick;
            
            if (!insidePolygon(clickPoint)) {
                
                // Store the second click as the end point point.
                if ( startHasBeenSet && robotEndPos.x == 0 && robotEndPos.y == 0 ) {
                    robotEndPos.x = clickPoint.x;
                    robotEndPos.y = clickPoint.y;
                }
                
                // Store the first click as the starting point.
                if (robotStartPos.x == 0 && robotStartPos.y == 0) {
                    robotStartPos.x = clickPoint.x;
                    robotStartPos.y = clickPoint.y;
                    startHasBeenSet = true;
                }
                
                
            } else {
                printf("Please click outside the polygons.");
            }
            // User is currently drawing polygon.
        } else {
            // Check if this is the first point drawn on canvas - if not, draw a line.
            if (polygonVertices.size() > 0) {
                // If user clicks back to the first point, he/she is finishing the polygon.
                point2D firstPoint = polygonVertices[0];
                if (withinRadius(mouseClick, firstPoint) && polygonSegments.size() > 2) {
                    mouseClick = firstPoint;
                    hasFinishedPolygon = 1;
                }
                
                // Last element in the vector is the previous point.
                segment2D currentLine = {polygonVertices.back(), mouseClick};
                // If line doesn't intersect other edges or if it's the last line.
                if (!intersectPolygon(currentLine) || hasFinishedPolygon) {
                    // Add new line.
                    polygonSegments.push_back(currentLine);
                    if (!hasFinishedPolygon) {
                        // Add new vertex.
                        polygonVertices.push_back(mouseClick);
                    } else{
                        printf("Push the polygon into vector of polygons.");
                        vector<segment2D> thisPolygon = polygonSegments;
                        polygons.push_back(thisPolygon);
                        polygonSegments.clear();
                        polygonVertices.clear();
                        hasFinishedPolygon = false;
                    }
                } else {
                    printf("Polygon cannot intersect itself.");
                }
            } else {
                // Add first point drawn on canvas.
                polygonVertices.push_back(mouseClick);
            }
        }
        // Update display.
        glutPostRedisplay();
    }
}




// Automatically move the Robot from start to finish.
void startMoving() {
    
    static int lastFrameTime = 0;
    int now, elapsed_ms;
    int i = moveForward  ? 0 : shortestpath.size() - 1;
    
    while (1) {
        //Only animate until it reaches the goal state
        if (i >= shortestpath.size()) {
            break;
        }
        
        now = glutGet (GLUT_ELAPSED_TIME);
        elapsed_ms = now - lastFrameTime;
        
        // Move visibility point every time interval.
        if (elapsed_ms  > 1) {
            
            robotPosition.x = shortestpath.at(i).x;
            robotPosition.y = shortestpath.at(i).y;
            robotAngle = shortestpathAngle.at(i);
            lastFrameTime = now;
            display();
            
            moveForward  ? i++ : i-- ;
            
            
        }
        
        
    }
}


/* ****************************** */
void display(void) {
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity(); //clear the matrix
    /* The default GL window is [-1,1]x[-1,1] with the origin in the
     center.
     The points are in the range (0,0) to (WINSIZE,WINSIZE), so they
     need to be mapped to [-1,1]x [-1,1]x */
    glScalef(2.0/WINDOWSIZE, 2.0/WINDOWSIZE, 1.0);
    glTranslatef(-WINDOWSIZE/2, -WINDOWSIZE/2, 0);
    
    
    // Draw polygons.
    drawPolygons();
    drawPolygonInProgress();
    drawStartEnd();
    drawPath();
    drawRobot(robotPosition.x,robotPosition.y,robotAngle);
    
    if (!hasFinishedPolygon) {
        drawCirclesAroundVertices();
    }
    
    /* execute the drawing commands */
    glFlush();
}

/* ****************************** */
void keypress(unsigned char key, int x, int y) {
    switch(key) {
        case 'q':
            // Exits the program.
            exit(0);
            break;
        case ' ':
            // Exits obstacle drawing mode.
            hasFinishedPolygons = true;
            polygons.clear();
            interestingTestCase();
             glutPostRedisplay();
          break;
        case 'e':
            // Exits obstacle drawing mode.
            hasFinishedPolygons = true;
            break;
        case 'm':
            //Draws shortest path
            if (shortestpath.empty()){
                shortestpath =  findShortestPath();
                // Starts automatic movements.
                startMoving();
                moveForward = false;
            } else{
                // Starts automatic movements.
                startMoving
                ();
                moveForward = true;
            }
            glutPostRedisplay();
            break;
            
    }
}

/* Handler for window re-size event. Called back when the window first appears and
 whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);
    
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    gluOrtho2D(0.0, (GLdouble) width, 0.0, (GLdouble) height);
}


void interestingTestCase(){
//    vector<segment2D> segments;
//    
//    segments.push_back({{61,87},{132,34}});
//    
//    segments.push_back({{132,34},{116,90}});
//    
//    segments.push_back({{116,90},{53,92}});
//    
//    segments.push_back({{53,92},{61,87}});
//    
//    vector<segment2D> thisPolygon = segments;
//    
//    polygons.push_back(thisPolygon);
//    
//    segments.clear();
//    
//    
//    segments.push_back({{99,152},{235,82}});
//    
//    segments.push_back({{235,82},{391,97}});
//    
//    segments.push_back({{391,97},{238,238}});
//    
//    segments.push_back({{238,202},{180,82507}});
//    segments.push_back({{180,250},{112,224}});
//    
//    segments.push_back({{112,224},{184,157}});
//        segments.push_back({{184,157},{99,152}});
//    
//    
//    
//     thisPolygon = segments;
//    
//    polygons.push_back(thisPolygon);
//    
//    segments.clear();
    
    
    vector<segment2D> segments;
    
    segments.push_back({{61,87},{132,34}});
    
    segments.push_back({{132,34},{116,90}});
    
    segments.push_back({{116,90},{53,92}});
    
    segments.push_back({{53,92},{61,87}});
    
    vector<segment2D> thisPolygon = segments;
    
    polygons.push_back(thisPolygon);
    
    segments.clear();
    
    
    segments.push_back({{99,152},{235,82}});
    
    segments.push_back({{235,82},{391,97}});
    
    segments.push_back({{391,97},{238,202}});
    
    segments.push_back({{238,202},{180,250}});
    segments.push_back({{180,250},{112,224}});
    
    segments.push_back({{112,224},{184,157}});
    segments.push_back({{184,157},{99,152}});
    
    
    
    thisPolygon = segments;
    
    polygons.push_back(thisPolygon);
    
    segments.clear();
    
    
    segments.push_back({{99,398},{125,320}});
    
    segments.push_back({{125,320},{281,252}});
    
    segments.push_back({{281,252},{297,342}});
    
    segments.push_back({{297,342},{199,373}});
    segments.push_back({{199,373},{278,412}});
    
    segments.push_back({{278,412},{150,431}});
    segments.push_back({{150,431},{99,398}});
    
    
    
    
    thisPolygon = segments;
    
    polygons.push_back(thisPolygon);
    
    segments.clear();
    robotStartPos = {386,358};
    robotEndPos = {49,185};
    
    startHasBeenSet = true;
    
    

}


    




