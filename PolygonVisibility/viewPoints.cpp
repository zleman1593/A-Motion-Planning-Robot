/* view.c
 
 Laura Toma, Ivy Xing, Zackery Leman
 
 What it does:
 
 
 */
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <queue>
#include <vector>
#include "geom.h"
#include "rtimer.h"
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


int ROBOT_SIZE = 5;

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
void erase(void);

/* Helper computation functions */
int intersectPolygon(segment2D current);
int insidePolygon(point2D p);
void clearVectors(void);



/**** Global variables ****/
int hasFinishedPolygon = 0; // Whether the user has finished drawing the polygon.
point2D visiblityPoint; // The point of visibility inside the polygon.
point2D firstClick = {0, 0};
point2D secondClick = {0, 0};
point2D robotPosition;
bool startHasBeenSet = false;

int constant = ROBOT_SIZE;

std::map<std::string, std::string> duplicateMap;

const int WINDOWSIZE = 500;
const int NUM_SEGMENTS = 800; // Number of segments that form the circle.


//Priority queue comparison function
class CompareStateDistance {
public:
    bool operator()(State& t1, State& t2)
    {
        if (t1.distance > t2.distance) return true;
        return false;
    }
};

//NOTE: all the structures below need to be global so that they can be rendered

// The array of segments that form the polygon.
vector<vector<segment2D>>  polygons;
vector<segment2D>  polygonSegments;
priority_queue<State,vector<State>,CompareStateDistance> nextStateQueue;


vector<point2D>  exploredVector;

// The vertices of the polygon.
vector<point2D> polygonVertices;
// The array of segments that form the visible area of the polygon.
vector<segment2D>  visibleAreaSegments;
// The slope of the visible area segments.
vector<double>visibleAreaSegmentSlopes;
// The vertices of the polygon that are visible from the visibility point.
vector<point2D> visibleAreaVertices;
// The intersection points of the extended lines with the egdes of the polygon.
vector<point2D> extendedLineIntersections;

vector<point2D> shortestpath;


//Check if any of the four sides of the rectangular (square) robot intersects any of the edges of the obstacles
void drawRobot(double robotCenterX, double robotCenterY){
    
    
    point2D bottomRight = {robotCenterX + constant, robotCenterY - constant};
    point2D bottomLeft =  {robotCenterX - constant, robotCenterY - constant};
    point2D topLeft =  {robotCenterX - constant, robotCenterY + constant};
    point2D topRight =  {robotCenterX + constant, robotCenterY + constant};
  
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

/*Method to prevent locations from being revisited. Done in linear time with hashmap*/
bool hasBeenToSate(double x, double y) {
    
    std::ostringstream oss;
    oss << floor(x) << " " << floor(y);
    std::string tempString = oss.str();
    
    // check if key is present
    if (duplicateMap.find(tempString) != duplicateMap.end()){
        //Map Already Contains Point
        //std::cout << "map already contains the point!\n";
        return true;
    } else{
        duplicateMap[tempString] = "";
        return false;
    }
}

//Returns  whether at goal state
bool atGoal(double robotCenterX, double robotCenterY){
    
    if (doubleEqual(robotCenterX,secondClick.x) && doubleEqual(robotCenterY,secondClick.y)) {
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
    
    point2D bottomRight = {robotCenterX + constant, robotCenterY - constant};
    point2D bottomLeft =  {robotCenterX - constant, robotCenterY - constant};
    point2D topLeft =  {robotCenterX - constant, robotCenterY + constant};
    point2D topRight =  {robotCenterX + constant, robotCenterY + constant};
    
    //First check if any of the points are out of bounds
    if (outOfBounds(bottomRight) || outOfBounds(bottomLeft) || outOfBounds(topLeft) || outOfBounds(topRight)){
        return true;
    }
    
    
    segment2D leftVert = {topLeft,bottomLeft};
    segment2D rightVert =  {topRight,bottomRight};
    segment2D top =  {topLeft,topRight};
    segment2D bottom =  {bottomLeft,bottomRight};
    
    
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

State* getState(double robotCenterX, double  robotCenterY,State robotState){
    
    
    if (!hasBeenToSate(robotCenterX, robotCenterY) && !robotHitsObstacle(robotCenterX, robotCenterY)) {
        
        State * nextPossibleState = new State();
        nextPossibleState->location = {robotCenterX, robotCenterY};
        nextPossibleState->path = robotState.path;
        nextPossibleState->path.push_back(nextPossibleState->location );
        nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
        
        exploredVector.push_back(nextPossibleState->location);
        
        nextStateQueue.push(*nextPossibleState);
        if (atGoal(robotCenterX, robotCenterY)) {
            return nextPossibleState;
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
    
    
    
    //Up Right
    state = getState(robotCenter.x + 1, robotCenter.y + 1,currentRobotState);
    
    if ( state != NULL) {
        return state;
    }
    
    //Down Right
    state = getState(robotCenter.x + 1, robotCenter.y - 1,currentRobotState);
    
    if ( state != NULL) {
        return state;
    }
    
    // Up Left
    state = getState(robotCenter.x + 1, robotCenter.y - 1,currentRobotState);
    if ( state != NULL) {
        return state;
    }
    
    // Down Left
    
    state = getState(robotCenter.x - 1, robotCenter.y - 1,currentRobotState);
    if ( state != NULL) {
        return state;
    }
    
    
    
    
    return NULL;
    
}

//Called to find the shortest path around the obstacles. Returns this path.
vector<point2D>  findShortestPath(){
    
    State * nextPossibleState = new State();
    nextPossibleState->location = firstClick;
    nextPossibleState->distance = distance_(nextPossibleState->location,secondClick);
    generateSuccessors(*nextPossibleState);
    hasBeenToSate(firstClick.x, firstClick.y);
    
    
    State currentState;
    while (!nextStateQueue.empty()) {
        
        currentState = nextStateQueue.top();
        nextStateQueue.pop();
        State *terminal = generateSuccessors(currentState);
        if (terminal != NULL) {
            return terminal->path;
        }
        
    }
    
    vector<point2D>  empty;
    return empty;
    
}




/* ****************************** */
int main(int argc, char** argv) {
    
    //Create Dummy Polygons
    
    
    
    
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
void drawPolygon(void) {
    glColor3fv(red);
    for (int i = 0; i < polygonSegments.size(); i++) {
        drawSegment(polygonSegments[i]);
    }
}


// Draw the start and end points.
void drawStartEnd(void) {
    if (startHasBeenSet) {
        glColor3fv(green);
        drawCircle(firstClick.x, firstClick.y, RADIUS, NUM_SEGMENTS, 1);
    }
    
    if (startHasBeenSet) {
        glColor3fv(red);
        drawCircle(secondClick.x, secondClick.y, RADIUS, NUM_SEGMENTS, 1);
    }
}


void drawExplored(){
    for (int i = 0; i < exploredVector.size(); i++) {
        glColor3fv(yellow);
        //glBegin(GL_POINT);
        drawCircle(exploredVector.at(i).x, exploredVector.at(i).y, 3, 20, 1);
        //glVertex2f(exploredVector.at(i).x, exploredVector.at(i).y);
        //glEnd();
    }
}



// Draw line segments to the visible verticies.
void drawVisibleAreaSegments(void) {
    for (int i = 0; i < visibleAreaSegments.size(); i++) {
        drawSegment(visibleAreaSegments[i]);
    }
}



// Erase the polygon.
void erase(void) {
    polygonSegments.clear();
    polygonVertices.clear();
    visiblityPoint = {-1, -1};
    
    // Clear computations.
    clearVectors();
    // Allow user to draw a new polygon.
    hasFinishedPolygon = 0;
    // Update dispplay.
    glutPostRedisplay();
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



// Clear all stored data; called when visibility point changes.
void clearVectors(void) {
    visibleAreaVertices.clear();
    visibleAreaSegments.clear();
    visibleAreaSegmentSlopes.clear();
    extendedLineIntersections.clear();
}

/**** Responsive and Display Functions ****/

// Respond to mouse clicks.
void mouse(int button, int state, int Mx, int My) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        point2D mouseClick = {(double)Mx, (double)(WINDOWSIZE - My)};
        
        // User finished drawing polygon, so compute visibility area.
        if (hasFinishedPolygon) {
            visiblityPoint = mouseClick;
            
            if (!insidePolygon(visiblityPoint)) {
                
                // Store the second click as the end point point.
                if ( startHasBeenSet && secondClick.x == 0 && secondClick.y == 0 ) {
                    secondClick.x = visiblityPoint.x;
                    secondClick.y = visiblityPoint.y;
                }
                
                // Store the first click as the starting point.
                if (firstClick.x == 0 && firstClick.y == 0) {
                    firstClick.x = visiblityPoint.x;
                    firstClick.y = visiblityPoint.y;
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
void startMoving(void) {
    static int lastFrameTime = 0;
    int now, elapsed_ms;
    int i = 0;
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
            lastFrameTime = now;
            display();
            
            i++;
            
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
    
    /* Draw the polygon and its visible area. */
    // Draw polygon.
    
    drawPolygon();
    drawStartEnd();
    drawPath();
    //drawExplored();//Debug
    drawRobot(robotPosition.x,robotPosition.y);
    
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
        case 'e':
            // Erases the polygon.
            erase();
            break;
        case 'd':
            polygons.push_back(polygonSegments);//DEBUG
            shortestpath =  findShortestPath();
            
            glutPostRedisplay();
            break;
        case 'm':
            // Starts automatic movements.
            startMoving();
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
