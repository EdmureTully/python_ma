#include "bounding_geometry.h"
#include "haptic_utils.h"

#include<math.h>
#include <iostream>


geometry::shape::shape()
{

}

geometry::shape::~shape()
{

}

Eigen::Vector3d geometry::shape::get_min_values() const
{
    return min_values_;
}

Eigen::Vector3d geometry::shape::get_max_values() const
{
    return max_values_;
}

std::vector<geometry::plane> geometry::shape::get_planes() const
{
    return planes_;
}



geometry::triangle::triangle()
{

}

geometry::triangle::triangle(std::vector<Eigen::Vector3d> &verts)
{
    static_assert (3, "Wrong number of points for triangle");

    vertices_ = verts;
    edge_vector_u_ = vertices_.at(0) - vertices_.at(1);
    edge_vector_v_ = vertices_.at(0) - vertices_.at(2);

}

geometry::triangle::~triangle()
{

}

geometry::geometry_vector::geometry_vector()
{

}

geometry::geometry_vector::geometry_vector(Eigen::Vector3d &p0, Eigen::Vector3d &p1):
    p0_(p0), p1_(p1)
{
    vector_ = p1_ - p0_;
    vector_.normalize();
}

geometry::geometry_vector::~geometry_vector()
{

}

Eigen::Vector3d geometry::geometry_vector::get_vector() const
{
    return vector_;
}

Eigen::Vector3d geometry::geometry_vector::get_p0() const
{
    return p0_;
}

Eigen::Vector3d geometry::geometry_vector::get_p1() const
{
    return p1_;
}

geometry::geometry_vector geometry::geometry_vector::calculate_cross_product(const geometry_vector& g_v)
{
    Eigen::Vector3d cross = g_v.get_vector().cross(vector_);
    cross.normalize();
    Eigen::Vector3d p0 = g_v.get_p0();
    Eigen::Vector3d p1 = p0 + cross;

    return geometry_vector(p0, p1);
}

geometry::plane::plane(double a, double b, double c, double d):
    a_(a), b_(b), c_(c), d_(d)
{

}

geometry::plane::plane(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2):
    p0_(p0),p1_(p1),p2_(p2)
{

    mid_vec_ = ((p1_ - p0_) + (p2_ - p0_)) * 0.5;
    geometry_vector v0(p0, p1);
    geometry_vector v1(p0, p2);
    geometry_vector normal = v0.calculate_cross_product(v1);

    a_ = normal.get_vector().x();
    b_ = normal.get_vector().y();
    c_ = normal.get_vector().z();
    d_ = -(p0.x() * a_ + p0.y() * b_ + p0.z() * c_);
}

geometry::plane::plane()
{

}

geometry::plane::~plane()
{

}

void geometry::plane::swap_sign_params()
{
    a_ *= -1.0;
    b_ *= -1.0;
    c_ *= -1.0;
    d_ = -(p0_.x() * a_ + p0_.y() * b_ + p0_.z() * c_);

}

double geometry::plane::calculate_distance_to_plane(const Eigen::Vector3d &point)
{
    //geometry_vector v()
    Eigen::Vector3d point1 = (point - p0_);
    double dist = a_ * point.x() + b_ * point.y() + c_ * point.z() + d_;
    return dist;
}

Eigen::Vector3d geometry::plane::get_normal_vector()
{
    return Eigen::Vector3d(a_, b_, c_);
}

double geometry::plane::get_d_component() const
{
    return d_;
}


//(x,y,z) = point + t[a,b,c]; d = a(t*a + p_x) + b*(t*b + p_y) + c*(t*c + p_z)
Eigen::Vector3d geometry::plane::get_closest_point_on_plane_to_point(Eigen::Vector3d &point)
{
    //calculate t parameter to get point on line = point on plane
    double nom = -1.0 * d_ - a_ * point.x() -b_ * point.y() - c_ * point.z();
    double denom = a_ * a_ + b_ * b_ + c_ * c_;
    double t = nom / denom;

    //calculate point on line
    double x = point.x() + t * a_;
    double y = point.y() + t * b_;
    double z = point.z() + t * c_;

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d geometry::plane::get_mid_vector()
{
    return mid_vec_;
}

geometry::cone::cone()
{

}

geometry::cone::cone(Eigen::Vector3d &tip, double height, double opening_angle, double radius, unsigned int b_points):
    tip_(tip), height_(height ), half_opening_angle_(opening_angle * M_PI / 180.0), radius_(radius), base_points_(b_points)
{
    if((half_opening_angle_ > 0.0)  && (radius_ < 0.0)) calculate_radius();

    if((half_opening_angle_ < 0.0) && (radius_ > 0.0)) calculate_opening_angle();


    balance_point_ = tip_;
    balance_point_.z() = tip_.z() + 2.0/3.0 * height_;

    calculate_base_points(base_points_);

    create_planes();

    create_polygon();

}

geometry::cone::~cone()
{

}

const geometry::polygon geometry::cone::get_polygon() const
{
  return cone_polygon_;
}

void geometry::cone::init_cone(Eigen::Vector3d &tip, double height, double opening_angle, double radius, unsigned int base_points)
{
  tip_ = tip;
  height_ = height ;
  half_opening_angle_ = opening_angle;
  radius_ = radius ;
  base_points_ = base_points;
  balance_point_ = tip_;
  balance_point_.z() = tip_.z() + 2.0/3.0 * height_;

  if((half_opening_angle_ > 0.0)  && (radius_ < 0.0)) calculate_radius();

  if((half_opening_angle_ < 0.0) && (radius_ > 0.0)) calculate_opening_angle();

  calculate_base_points(base_points_);

  create_planes();

  create_polygon();
}

Eigen::Vector3d geometry::cone::get_balance_point() const
{
  return balance_point_;
}

std::vector<Eigen::Vector3d> geometry::cone::get_base_points() const
{
  return pts_on_circle_;
}

std::vector<geometry::plane> geometry::cone::get_planes()
{
  return planes_;
}



void geometry::cone::calculate_radius()
{
    radius_ = height_ * std::tan(half_opening_angle_);
}

void geometry::cone::calculate_opening_angle()
{
    half_opening_angle_ = std::atan(radius_ / height_);
}

std::vector<Eigen::Vector3d> geometry::cone::calculate_base_points(unsigned int num_points)
{
    double rad  = static_cast<double>((( 2 * M_PI) / num_points));
    std::vector<Eigen::Vector3d> pts_on_circle;

    for (unsigned int i = 0; i < num_points; i++) {

        double x = std::cos(rad * i) * radius_ + tip_.x();
        double y = std::sin(rad * i) * radius_ + tip_.y();
        double z = tip_.z() + height_;

        Eigen::Vector3d point_on_circle(x, y, z);

        //haptic_utils::write_eigen_vector3_to_console(point_on_circle);
        pts_on_circle.push_back(point_on_circle);
    }

    pts_on_circle_ = pts_on_circle;
    return pts_on_circle;
}

void geometry::cone::create_polygon()
{
    //counter, every third point in the polygon should be the tip
    unsigned int counter(1);

    std::vector<Eigen::Vector3d> pol_pts;

    for (const Eigen::Vector3d& point : pts_on_circle_) {
        pol_pts.push_back(point);




        pol_pts.push_back(tip_);

        pol_pts.push_back(pts_on_circle_.at(counter  % (pts_on_circle_.size())));

        counter++;
    }

    cone_polygon_ = polygon(pol_pts);
}

void geometry::cone::create_planes()
{
    std::vector<Eigen::Vector3d>::iterator pts_iter(pts_on_circle_.begin()),pts_next(pts_iter++), pts_end(pts_on_circle_.end());

    //Masseschwerpunkt des Kegels
    Eigen::Vector3d mass_center = Eigen::Vector3d(tip_.x(), tip_.y(), tip_.z() + height_ * 0.75);


    for(; pts_iter != pts_end + 1; pts_iter++){

       if(pts_next == pts_on_circle_.end()-1){
            pts_next = pts_on_circle_.begin();
        }

        plane temp_plane(*pts_iter, *pts_next, tip_);

        double distance = temp_plane.calculate_distance_to_plane(mass_center);

        if(distance > 0.1){
            temp_plane.swap_sign_params();
        }

        pts_next++;

        planes_.push_back(temp_plane);
    }

    //Oberflächen(Kreisfläche berechnen)


    Eigen::Vector3d p0(pts_on_circle_[0]), p1(pts_on_circle_[1]), p2(pts_on_circle_[2]);

    plane circle_plane(p0, p1, p2);

    double distance = circle_plane.calculate_distance_to_plane(mass_center);

    if(distance > 0.1){
        circle_plane.swap_sign_params();
    }

    pts_next++;

    planes_.push_back(circle_plane);
}

geometry::face::face()
{

}

geometry::face::face(std::vector<Eigen::Vector3d> &face_points, std::vector<unsigned int> &indices):
    face_pts_(face_points), indices_(indices), num_vertices_(face_points.size())
{

}

std::vector<Eigen::Vector3d> geometry::face::get_face_points() const
{
    return face_pts_;
}

std::vector<unsigned int> geometry::face::get_indices() const
{
    return indices_;
}

unsigned int geometry::face::get_number_vertices() const
{
    return num_vertices_;
}

geometry::polygon::polygon()
{

}

geometry::polygon::polygon(std::vector<Eigen::Vector3d> &pol_pts):
    pol_pts_(pol_pts), num_vertices_(pol_pts.size())
{
    for(unsigned int i = 0; i < pol_pts.size(); i++){
        indices_.push_back(i);
    }
}

geometry::polygon::~polygon()
{

}

std::vector<Eigen::Vector3d> geometry::polygon::get_polygon_points() const
{
    return pol_pts_;
}

std::vector<unsigned int> geometry::polygon::get_indices() const
{
    return  indices_;
}

unsigned int geometry::polygon::get_number_of_vertices() const
{
    return num_vertices_;
}

geometry::bounding_box::bounding_box()
{

}

geometry::bounding_box::bounding_box(std::vector<Eigen::Vector3d> &points):
    box_points_(points)
{
    create_planes();
}

geometry::bounding_box::bounding_box(Eigen::Vector3d &center, double height, double width, double length):
    center_(center), height_(height), width_(width), length_(length)
{
    calculate_box_points();
    create_planes();
}

geometry::bounding_box::~bounding_box()
{

}

void geometry::bounding_box::update_bounding_box(Eigen::Vector3d &center, double height, double width, double length)
{
  center_ = center;
  height_ = height;
  width_ = width;
  length_ = length;
  calculate_box_points();
  create_planes();
}

std::vector<Eigen::Vector3d> geometry::bounding_box::get_box_points() const
{
  return box_points_;
}



void geometry::bounding_box::create_planes()
{

    plane p0 (box_points_[0], box_points_[1], box_points_[3]);//bottom checked
    plane p1 (box_points_[0], box_points_[3], box_points_[4]);//left checked
    plane p2 (box_points_[1], box_points_[2], box_points_[5]);//right checked
    plane p3 (box_points_[2], box_points_[3], box_points_[6]);//front checked
    plane p4 (box_points_[4], box_points_[5], box_points_[7]);//top checked
    plane p5 (box_points_[0], box_points_[1], box_points_[4]);//back checked

    planes_.push_back(p0);
    planes_.push_back(p1);
    planes_.push_back(p2);
    planes_.push_back(p3);
    planes_.push_back(p4);
    planes_.push_back(p5);

    //calculoate distance between the planes and the box's center
    //if positive, swap the signs of the plane's coordinate form so the all point to the same direction
    for(plane& pl : planes_){
        double dist = pl.calculate_distance_to_plane(center_);
        if(dist > 0.00001){
           pl.swap_sign_params();
           std::cout <<"Param swapped" << std::endl;
        }
    }
}

void geometry::bounding_box::calculate_box_points()
{
    //see header for the numeration of the vertices
    Eigen::Vector3d p0 = center_ + Eigen::Vector3d(-width_/2, length_/2, -height_/2);
    Eigen::Vector3d p1 = center_ + Eigen::Vector3d(width_/2, length_/2, -height_/2);
    Eigen::Vector3d p2 = center_ + Eigen::Vector3d(width_/2, -length_/2, -height_/2);
    Eigen::Vector3d p3 = center_ + Eigen::Vector3d(-width_/2, -length_/2, -height_/2);
    Eigen::Vector3d p4 = center_ + Eigen::Vector3d(-width_/2, length_/2, height_/2);
    Eigen::Vector3d p5 = center_ + Eigen::Vector3d(width_/2, length_/2, height_/2);
    Eigen::Vector3d p6 = center_ + Eigen::Vector3d(width_/2, -length_/2, height_/2);
    Eigen::Vector3d p7 = center_ + Eigen::Vector3d(-width_/2, -length_/2, height_/2);

    box_points_.push_back(p0);
    box_points_.push_back(p1);
    box_points_.push_back(p2);
    box_points_.push_back(p3);
    box_points_.push_back(p4);
    box_points_.push_back(p5);
    box_points_.push_back(p6);
    box_points_.push_back(p7);
}

geometry::geometry_procedures::geometry_procedures()
{

}

geometry::geometry_procedures::~geometry_procedures()
{

}

bool geometry::geometry_procedures::point_inside_object(geometry::shape &shape, Eigen::Vector3d &pt)
{

    for(plane& pl : shape.get_planes()){
        double distance = pl. calculate_distance_to_plane(pt);

        if(distance > 0.000001){
            //std::cout << "Punkt nicht in Koerper" << std::endl;
            return false;
        }
    }

    return true;
}

bool geometry::geometry_procedures::point_inside_object(geometry::shape &shape, Eigen::Vector3d &pt, geometry::plane &closest_plane)
{
    bool inside(true);
    double closest_distance(1000.0);

    //go over every plane of object and check, on which side the point lies
    for(plane& pl : shape.get_planes()){
        double distance = pl. calculate_distance_to_plane(pt);

        //std::cout << std::to_string(distance) << std::endl;

        //point lies outside of object if positive
        if(distance > 0.000001){
            std::cout << "Punkt nicht in Koerper" << std::endl;
            inside = false;
            break;
        }

        //if distance is smaller than current smallest distance, set the new closest plane
        if(std::abs(distance) < closest_distance){
            closest_plane = pl;
            closest_distance = std::abs(distance);   
        }
    }

    return inside;
}


Eigen::Vector3d geometry::get_absolut_max_component_vector(Eigen::Vector3d &vec)
{
    double x = std::abs(vec.x());
    double y = std::abs(vec.y());
    double z = std::abs(vec.z());


}
