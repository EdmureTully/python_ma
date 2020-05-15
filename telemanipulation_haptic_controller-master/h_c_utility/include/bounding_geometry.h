#ifndef BOUNDING_GEOMETRY_H
#define BOUNDING_GEOMETRY_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace  geometry{

//util functions

Eigen::Vector3d get_absolut_max_component_vector(Eigen::Vector3d& vec);


class plane;

class polygon{
public:

    /**
    * @brief standard constructor
    */
    polygon();

    /**
    * @brief constructor
    * @param pol_poits the points that make the polygon
    */
    polygon(std::vector<Eigen::Vector3d>& pol_pts);

    /**
    * @brief deconstructor
    */
    ~polygon();


    //--------------------------getter-----------------------------------

    /**
    * @brief get_polygon_points returns the polygon's points(vertices)
    * @return polygon's points(vertices)
    */
    std::vector<Eigen::Vector3d> get_polygon_points()const;

    /**
    * @brief get_indices returns the point's indices
    * @return indices
    */
    std::vector<unsigned int> get_indices()const;

    /**
    * @brief get_number_of_vertices returns the number of vertices of the polygon
    * @return number of polygon's vertices
    */
    unsigned int get_number_of_vertices()const;


private:

    std::vector<Eigen::Vector3d> pol_pts_;/**< polygon's points */

    std::vector<unsigned int> indices_;/**< indices of the points*/

    unsigned int num_vertices_;/**< numer of points(vertices)*/

};


class shape
{
public:

    /**
    * @brief standard constructor
    */
    shape();

    /**
    * @brief standard deconstructor
    */
    virtual ~shape();

    /**
    * @brief get_min_value returns the minimal values in x,y and z direction in one vector
    */
    virtual Eigen::Vector3d get_min_values() const;

    /**
    * @brief get_max_value returns the maximal values in x,y and z direction in one vector
    */
    virtual Eigen::Vector3d get_max_values() const;

    /**
    * @brief get_planes returns the planes that define a shape
    * @return planes in a vector
    */
    virtual std::vector<plane> get_planes() const;

    const polygon get_polygon()const;

protected:


    Eigen::Vector3d min_values_; /**< minimal values in x,y,z direction */
    Eigen::Vector3d max_values_;/**< maximal values in x,y,z direction*/
    std::vector<plane> planes_;/**< planes the shape consists of*/
    polygon polygon_;
};


/**
* @brief The geometry vector class represents a vector consisting of two points
*/
class geometry_vector
{

public:

    /**
    * @brief standard constructor
    */
    geometry_vector();

    /**
    * @brief constructor
    * @param p0 starting point of vector
    * @param p1 end point of vector
    */
    geometry_vector(Eigen::Vector3d& p0, Eigen::Vector3d& p1);

    /**
    * @brief deconstructor
    */
    ~geometry_vector();

    /**
    * @brief calculate_cross_product calculates the cross product between this vector and a given second one
    * @param g_v second geometry_vector to calculate tthe cross product with
    * @return cross product result as geometry_vector
    */
    geometry_vector calculate_cross_product(const geometry_vector& g_v);


    //--------------------------getter-----------------------------------

    /**
    * @brief get_vector returns the vector(p1-p0)
    * @return vector
    */
    Eigen::Vector3d get_vector()const;

    /**
    * @brief get_p0 return the starting point of the vector
    * @return starting point
    */
    Eigen::Vector3d get_p0()const;

    /**
    * @brief get_p1 returns the end point of the vector
    * @return end_point of the vector
    */
    Eigen::Vector3d get_p1()const;

private:

    Eigen::Vector3d p0_;/**< starting point */
    Eigen::Vector3d p1_;/**< ending point*/
    Eigen::Vector3d vector_;/**< vector (p1-p0) */

};

class triangle
{
public:

    /**
    * @brief
    */
    triangle();

    /**
    * @brief
    */
    triangle(std::vector<Eigen::Vector3d>& verts);

    /**
    * @brief
    */
    ~triangle();

private:

    std::vector<Eigen::Vector3d> vertices_;/**< */
    Eigen::Vector3d edge_vector_u_;/**< */
    Eigen::Vector3d edge_vector_v_;/**< */

};

class plane{
public:

    //plane equation A * x + B *y + C * z + D

    /**
    * @brief constructor takes the parameters of the coordinate form as input
    * @param a component
    * @param b component
    * @param c component
    * @param d component
    */
    plane(double a, double b, double c, double d);

    /**
    * @brief constructor takes three points(not vectors) that define the plane and calculates the coordinate form
    * @param p0 first point
    * @param p1 second point
    * @param p2 third point
    */
    plane(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2);

    /**
    * @brief standard constructor
    */
    plane();

    /**
    * @brief deconstructor
    */
    ~plane();

    /**
    * @brief swap_sign_params swaps the sign of the  plane's coordinate form components
    * @param plane with swapped components
    */
    void swap_sign_params();

    /**
    * @brief calculate_distance_to_plane calculates the distance of a point to this plane
    * @return distance
    */
    double calculate_distance_to_plane(const Eigen::Vector3d& point);

    /**
    * @brief get_normal_vector returns the normal vector of the plane(a,b,c component)
    * @return normal vector
    */
    Eigen::Vector3d get_normal_vector();

    /**
    * @brief get_d_component
    * @return d component of plane equation
    */
    double get_d_component()const;

    /**
    * @brief get_closest_point_plane_to_point calculates the closest point on the plane in relation to another point
    * @param point point to which the closest pont on plane shoulkd be calculated
    * @return closest point on plane
    */
    Eigen::Vector3d get_closest_point_on_plane_to_point(Eigen::Vector3d& point);

    Eigen::Vector3d get_mid_vector();

private:

    Eigen::Vector3d p0_;/**< */
    Eigen::Vector3d p1_;/**< */
    Eigen::Vector3d p2_;/**< */
    Eigen::Vector3d mid_vec_;
    double a_;/**< a component of coordinate form*/
    double b_;/**< b component of coordinate form*/
    double c_;/**< c component of coordinate form*/
    double d_;/**< d component of coordinate form*/
};

class face{
public:

    /**
    * @brief standard constructor
    */
    face();

    /**
    * @brief constructor
    * @param face_points points, the face consists of
    * @param indices points' indices
    */
    face(std::vector<Eigen::Vector3d> &face_points, std::vector<unsigned int> &indices);

    /**
    * @brief deconstructor
    */
    ~face(){}

    //--------------------------getter-----------------------------------
    /**
    * @brief get_face_points returns the face's points
    * @return face's points
    */
    std::vector<Eigen::Vector3d> get_face_points()const;

    /**
    * @brief get_indices returns the face's points' indices
    * @return ace's points' indices
    */
    std::vector<unsigned int> get_indices()const;

    /**
    * @brief get_number_of_vertices returns the number of vertices the plane consists of
    * @return number of vertices
    */
    unsigned int get_number_vertices()const;

private:

    std::vector<Eigen::Vector3d> face_pts_;/**< all points the face consists of */
    std::vector<unsigned int> indices_;/**< indices of the points */
    unsigned num_vertices_;/**< number of points(vertices) */

};




class cone : public shape
{

public:

    /**
    * @brief standard constructor
    */
    cone();

    /**
    * @brief constructor
    * @param tip coordinate of the cone's tip in 3d
    * @param height height(length) of cone
    * @param opening_angle half angle of the cone
    * @param radius radius of the cone's circular surface
    * @param num_base_points number of points, the circular surface shall be divided in(equals number of planes the cone is represented as)
    */
    cone(Eigen::Vector3d& tip, double height, double opening_angle = -1.0, double radius = -1.0, unsigned int num_base_points = 0);

    /**
    * @brief deconstructor
    */
    ~cone();

    //--------------------------getter-----------------------------------

    /**
    * @brief get_polygon returns the polygon
    * @return polygon
    */
    const polygon get_polygon()const;


    /**
    * @brief get_polygon returns the polygon
    * @param tip cone's tip
    * @param height cone's height
    * @param opening_angle half opening angle
    * @param radius cone's base radius
    * @param base_points numer of points the cone's base is divided in
    */
    void init_cone(Eigen::Vector3d &tip, double height, double opening_angle, double radius, unsigned int base_points);

    /**
    * @brief get_polygon returns the polygon
    * @return polygon
    */
    Eigen::Vector3d get_balance_point()const;

    std::vector<Eigen::Vector3d> get_base_points()const;

    std::vector<plane> get_planes();

private:

    /**
    * @brief calculate_radius calculates the radius of the cone; needs height opening angle
    */
    void calculate_radius();

    /**
    * @brief calculate_opening_angle calculates the half angle of the cone; needs radius and height
    */
    void calculate_opening_angle();

    /**
    * @brief calculate_base_points calculates the points that lay on the circular surface of the cone(needed to represent cone via planes)
    * @param num_points number of parts the the cone should be divided in
    * @return vector containing the circular points
    */
    std::vector<Eigen::Vector3d> calculate_base_points(unsigned int num_points);

    /**
    * @brief create_polygon creates a polygon representation of the cone
    */
    void create_polygon();

    /**
    * @brief create_planes creates planes that represent the cone
    */
    void create_planes();

    Eigen::Vector3d tip_;/**< cone's tip*/
    Eigen::Vector3d balance_point_;/**< cone's balance point*/

    double height_;/**< */
    double half_opening_angle_;/**< cone's half opening angle*/
    double radius_;/**< radius*/
    unsigned int base_points_;/**< number of planes the cone will be divided into*/
    std::vector<Eigen::Vector3d> pts_on_circle_;/**< points on the circular surface*/

    polygon cone_polygon_;/**< the cone represented as polygon*/

};



/* Cube order
   4----------5
  /|         /|
 / |        / |
7--|-------6  |
|  |       |  |
|  0-------|--1
| /        | /
|/         |/
3----------2
*/

class bounding_box: public shape{
public:

    /**
    * @brief standard constructor
    */
    bounding_box();

    /**
    * @brief constructor
    * @param points corner points of the bounding box; should be 8
    */
    bounding_box(std::vector<Eigen::Vector3d>& points);

    /**
    * @brief constructor
    * @param center center point of the bounding box
    * @param height height of bounding box (z-coordinate)
    * @param width width of bounding box (x-coordinate)
    * @param length length of bounding box (y-coordinate)
    */
    bounding_box(Eigen::Vector3d& center, double height, double width, double length);

    /**
    * @brief deconstructor
    */
    ~bounding_box();


    /**
    * @brief updates the members for creating a new bounding  box
    * @param center box's center
    * @param height box's height
    * @param width box's width
    * @param length box's length
    */
    void update_bounding_box(Eigen::Vector3d& center, double height, double width, double length);

    //--------------------------getter-----------------------------------


    /**
    * @brief get_box_points returns the 8 points defining the bounding box
    * @return the 8 points in a vector
    */
    std::vector<Eigen::Vector3d> get_box_points()const;

private:

    /**
    * @brief create_planes creates the 6 planes defining the bounding box; box_points_ needed for this
    */
    void create_planes();

    /**
    * @brief calculate_box_points calculates the 8 points defining the box via the center point, length, width & height
    */
    void calculate_box_points();

    std::vector<Eigen::Vector3d> box_points_;/**< corner points of the bounding box (8)*/
    Eigen::Vector3d center_;/**< center of the bounding box as point*/
    double height_;/**< height of bouding box(z-coordinate)*/
    double width_;/**< width of bounding box(x-coordinate)*/
    double length_;/**< length of bounding box(y-coordinate)*/

};

class geometry_procedures{
public:

    /**
    * @brief standad constructor
    */
    geometry_procedures();

    /**
    * @brief standard deconstructor
    */
    ~geometry_procedures();

    /**
    * @brief point_inside_object checks if a given point lies in a given object; returns true if so, falls otherwise
    * @param shape object like cone, bounding box
    * @param pt point to be checked
    */
    bool point_inside_object(shape& shape, Eigen::Vector3d& pt);

    /**
    * @brief point_inside_object checks if a given point lies in a given object; returns true if so, falls otherwise
    * @param shape object like cone, bounding box
    * @param pt point to be checked
    * @param closest_plane return reference to the closest plane to the given point
    */
    bool point_inside_object(shape& shape, Eigen::Vector3d& pt, plane& closest_plane);




private:




};

}


#endif // BOUNDING_GEOMETRY_H
