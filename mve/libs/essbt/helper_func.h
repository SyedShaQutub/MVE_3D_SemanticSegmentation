
#include "defines.h"
#include "mve/mesh.h"
#include "mve/camera.h"
#include "mve/bundle.h"
#include "math/matrix.h"
#include "mve/scene.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

ESSBT_NAMESPACE_BEGIN

#define INVALID_LABEL 0
struct AppSettings
{
    std::string in_mesh;
    std::string out_mesh;
    std::string scene_path;
    uint funtionality; //1 == process ply to obj file; 0 == limit ply file
    // bool clean_degenerated = true;
    // bool delete_scale = false;
    // bool delete_conf = false;
    // bool delete_colors = false;
    // float conf_threshold = 1.0f;
    // float conf_percentile = -1.0f;
    // int component_size = 1000;
};

//void fill_viewing_direction(mve::CameraInfo cam, float* viewdir_vec);

struct facecam_{
    public:
    std::vector<unsigned int> camids;
    ushort major_label;
    std::vector<std::vector<math::Vec2f>> pixel_cordinates; /* cam ids and pixel co-ordinates correspond respectively to each other */
    unsigned long int face_id;
    facecam_(unsigned long int faceid, std::vector<unsigned int> cams, std::vector<std::vector<math::Vec2f>> pixelcordinates, ushort major_label ): face_id(faceid),camids(cams), pixel_cordinates(pixelcordinates), major_label(major_label) {}
    facecam_(){}
    bool operator() (facecam_ i,facecam_ j) { return (i.face_id<j.face_id);}
};

bool mesh_sanity_check(essbt::AppSettings conf);
bool mesh_sanity_check_1(essbt::AppSettings conf);
void normVec_face(mve::TriangleMesh::Ptr mesh, float* vec, std::size_t id);

essbt::facecam_ facecam_Superpos(mve::TriangleMesh::Ptr& mesh, mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views, std::size_t id );
essbt::facecam_ facecam_Superpos_new(mve::TriangleMesh::Ptr& mesh, mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views, std::vector<cv::Mat> const& image_list, std::size_t id );


ushort seman_labels( std::string const& image_path, std::vector <math::Vec2f> const image_points );
ushort seman_labels_new( std::string const& image_path, std::vector <math::Vec2f> const image_points, cv::Mat const& image);


ushort majority_element_moore_voting_algorithm(std::vector<ushort> pixel_vals);

std::vector<cv::Mat> image_read(mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views);
void show_image(std::vector<cv::Mat> image_list, uint index);


ESSBT_NAMESPACE_END
