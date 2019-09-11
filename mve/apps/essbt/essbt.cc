/*
 * Copyright (C) 2019, Syed Sha Qutub
 * Intel Labs
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 *
 * App to embed semantic segmentation information onto 3D triangles by back tracing.
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "essbt/defines.h"
#include "essbt/helper_func.h"
#include "util/arguments.h"
#include "mve/mesh.h"
#include "mve/mesh_io.h"
#include "mve/mesh_io_ply.h"
#include "mve/scene.h"
#include "mve/view.h"
#include "opencv2/opencv.hpp"
#include "tbb/tbb.h"
#include "omp.h"
#include <time.h>
#include "util/timer.h"
using namespace cv;

int
main (int argc, char** argv)
{
    //util::system::register_segfault_handler();
    //util::system::print_build_timestamp("MVE FSSR Mesh Cleaning");

/* Setup argument parser. */
    clock_t tStart = clock();
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_minnum(2);
    args.set_nonopt_maxnum(5);
    args.set_helptext_indent(25);
    args.set_usage(argv[0], "[ OPTS ] IN_MESH OUT_MESH");
    args.set_description("The application cleans degenerated faces resulting "
        "from MC-like algorithms. Vertices below a confidence threshold and "
        "vertices in small isolated components are deleted as well.");
    args.add_option('f', "functinality", true, "functinality 1=ply file proc; 0=limit file");
    args.parse(argc, argv);

    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    /* Init default settings. */
    essbt::AppSettings conf;
    conf.in_mesh = args.get_nth_nonopt(0);
    conf.out_mesh = args.get_nth_nonopt(1);
    conf.scene_path = args.get_nth_nonopt(2);
    struct essbt::facecam_ Facecam;

    while (util::ArgResult const* arg = args.next_result())
    {
    if (arg->opt == nullptr)
        continue;

    switch (arg->opt->sopt)
    {
        case 'f': conf.funtionality = arg->get_arg<uint>(); break;
        default:
            std::cerr << "Invalid option: " << arg->opt->sopt << std::endl;
            return EXIT_FAILURE;
    }
    }

    //sanity check
    mve::TriangleMesh::Ptr mesh;
    //bool valid_mesh = essbt::mesh_sanity_check(conf);
    bool valid_mesh = essbt::mesh_sanity_check_1(conf);
    //std::cout<<"\n\n Valid Mesh \n"<<valid_mesh;
    /* Load input mesh. */
    
    std::cout<<"\n\n inside mesh_sanity_check \n\n";
    mesh = mve::geom::load_mesh(conf.in_mesh);
    /* Sanity checks. */
    if (mesh->get_vertices().empty())
    {
        std::cerr << "Error: Mesh is empty!" << std::endl;
        return EXIT_FAILURE;
    }

    if (mesh->get_faces().empty())
    {
        std::cerr << "Error: Components/faces cleanup "
            "requested, but mesh has no faces." << std::endl;
        return EXIT_FAILURE;
    }
    //sanity check
    
    std::cout << "Loading mesh complete " << conf.in_mesh << std::endl;
    
    std::cout << "mesh faces " << mesh->get_faces().size() << std::endl;
    
    std::cout << "mesh vertices " << mesh->get_vertices().size() << std::endl;

    //read the views
    mve::Scene::Ptr scene;
    mve::Bundle::ConstPtr bundle;
    try
    {
    std::cout<<"\n Scene loading \n";
    scene = mve::Scene::create(conf.scene_path);
    bundle = scene->get_bundle(); 
    }
    catch (std::exception& e)
    {
        std::cerr << "Error loading scene: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    mve::Bundle::Cameras const& bundle_cams = bundle->get_cameras();
    mve::Scene::ViewList& views = scene->get_views();
    std::cout << " bundle size is : " << bundle_cams.size() << std::endl;
    //std::vector<essbt::facecam_> facecam;
    tbb::concurrent_vector<essbt::facecam_> facecam;

    std::size_t face_id_ = 0;    
    bool test = false;
    if(test == true)
    {
        mve::TriangleMesh::FaceList faces = mesh->get_faces();
        mve::MeshBase::VertexList vertices = mesh->get_vertices();
        mve::Bundle::Cameras const& bundle_cams = bundle->get_cameras();

        math::Vec3f const& a_ = vertices[102170];
        math::Vec3f const& b_ = vertices[100994];
        math::Vec3f const& c_ = vertices[101765];

        bool flag = false;
        while(flag == false){
            std::size_t ia = faces[face_id_ + 0];
            std::size_t ib = faces[face_id_ + 1];
            std::size_t ic = faces[face_id_ + 2];  

            math::Vec3f const& a = vertices[ia];
            math::Vec3f const& b = vertices[ib];
            math::Vec3f const& c = vertices[ic];

            if(((a_ == a) && (b_ == b) && (c_ == c)) || ((a_ == b) && (b_ == c) && (c_ == a)) || ((a_ == c) && (b_ == a) && (c_ == b)) ) {
                flag = true;
            }
            else
                ++face_id_;
        }

        //std::size_t face_id = 342421;
        std::size_t ia = faces[face_id_ + 0];
        std::size_t ib = faces[face_id_ + 1];
        std::size_t ic = faces[face_id_ + 2];  

        math::Vec3f const& a = vertices[ia];
        math::Vec3f const& b = vertices[ib];
        math::Vec3f const& c = vertices[ic];


        std::cout<<"\n\n Face id is : "<< face_id_ << "\n";              

        std::cout<<"\n\n Vertex id is : "<< ia;  
        std::cout<<"\n\n Vertex id is : "<< ib;                      
        std::cout<<"\n\n Vertex id is : "<< ic;

        std::cout<<"\n\n Vector id is : "<< a;  
        std::cout<<"\n\n Vector id is : "<< b;                      
        std::cout<<"\n\n Vector id is : "<< c<<"\n";      

        std::cout<<"\n\n Vector id a_ is : "<< a_;  
        std::cout<<"\n\n Vector id b_ is : "<< b_;                      
        std::cout<<"\n\n Vector id c_ is : "<< c_<<"\n"; 

    
        //return 0;

    }

    /* enable this  */
    util::WallTimer timer;    
    if (conf.funtionality == true){
        std::vector<cv::Mat> const& image_list = essbt::image_read(bundle, views);
        //essbt::show_image(image_list, 20);
        int mesh_facesize = mesh->get_faces().size();
        std::size_t meshparser_init = mesh_facesize*0;
        std::size_t meshparser_end = mesh_facesize;
        //facecam.grow_to_at_least(meshparser_end/3);
        int nProcessors = omp_get_max_threads();
        std::cout<<nProcessors<<std::endl;
        std::cout<<meshparser_end<<std::endl;
        omp_set_num_threads(nProcessors);
        std::size_t invalid_facecount = 0;
        #pragma omp parallel for num_threads(nProcessors)
        for (std::size_t i = meshparser_init; i < meshparser_end; i += 3)    
        // for (std::size_t i = 0; i < 1; i += 3)
        {

            //std::cout<<"\n\n face_id : "<< i;
            essbt::facecam_ facecam_superpos = essbt::facecam_Superpos_new(mesh, bundle, views, image_list, i);
            facecam.push_back(facecam_superpos);
            if(facecam_superpos.major_label == INVALID_LABEL)
                ++invalid_facecount ;
            
            if((facecam.size() % 500) == 0){
                float percentage = (facecam.size()*3.0*100.0)/meshparser_end;
                std::cout << "\n process status: : " << percentage << std::endl;
                std::cout << "\n Invalid face count: : " << invalid_facecount << std::endl;
                std::cout <<" took " << timer.get_elapsed_sec() << " sec." << std::endl;
            }
             //ushort label = facecam_superpos.major_label;  
            //facecam.at(i) = facecam_superpos;
            //#pragma omp ordered   
        }
        
        mve::TriangleMesh::ColorList& vectex_colors = mesh->get_vertex_colors();
        mve::TriangleMesh::ColorList& face_colors = mesh->get_face_colors();
        mve::MeshBase::VertexList& vertices = mesh->get_vertices();
        
        std::cout << "\nmesh face size /3 : " << mesh->get_faces().size()/3 << std::endl;
        std::cout << "mesh face size : " << mesh->get_faces().size() << std::endl;    
        std::cout << "mesh vertex size " << mesh->get_vertices().size() << std::endl;
        std::cout << "mesh vertex color size " << mesh->get_vertex_colors().size() << std::endl;
        std::cout << "mesh face color size " << mesh->get_face_colors().size() << std::endl;    
        std::cout << "\n Invalid face count: : " << invalid_facecount << std::endl;
     
        std::sort(facecam.begin(), facecam.end(), Facecam);
        for (std::size_t i = 0; i < facecam.size(); ++i)    
        //for (std::size_t i = 0; i < 1; i += 3)
        {

            essbt::facecam_ faceCam = facecam.at(i);
            std::cout<<"\n face cam face id is : "<<faceCam.face_id;            
            ushort label =  faceCam.major_label;
            math::Vec4f ColorList{label,label,label,label};
            face_colors.push_back(ColorList);
            //std::cout<<"face color : " << i << " " << vectex_colors.at(i) << std::endl;
        }

        if(facecam.size() == mesh_facesize/3){
            std::cout<<"\n Algorithm works\n";
        }
        else
        {
            std::cout<<"\n facecam size : "<<facecam.size() <<std::endl;
            std::cout<<"\n facecam size : "<<mesh_facesize << std::endl;
            std::cout<<"\n Check the algorithm\n";
        }
    }
    /* enable this  */

/* limiting mesh for testing purpose  */    

    else if (conf.funtionality == false){
        std::size_t num_vertices = mesh->get_vertices().size();
        mve::TriangleMesh::DeleteList delete_verts(num_vertices, true);
        std::cout << "I is initialised "<< std::endl;
        uint initial_iter = num_vertices*0;//770015, 370015
        uint end_iter = num_vertices*0.02; //970030, 670030
        std::cout << "initial_iter : "<<initial_iter<< std::endl;
        std::cout << "end_iter : "<<end_iter<< std::endl;    
        for (std::size_t i = initial_iter; i < end_iter; ++i){
            std::cout << "I is " << i<< std::endl;
            delete_verts[i] = false;  /* retaining these vertices */
        }   
        mesh->delete_vertices_fix_faces(delete_verts);
    }

/* limiting mesh for testing purpose  */    

/* Write output mesh. */
    /* ply/obj format */
    mve::geom::SavePLYOptions ply_opts;
    ply_opts.write_vertex_colors = true;
    ply_opts.write_vertex_confidences = true;
    ply_opts.write_vertex_values = true;
    std::cout << "Mesh output file: " << conf.out_mesh << std::endl;
    //mve::geom::save_ply_mesh(mesh, conf.out_mesh, ply_opts);
    mve::geom::save_mesh(mesh, conf.out_mesh);
    /* ply/obj format */

    
    double elapsed_time = (clock() - tStart)/CLOCKS_PER_SEC;
    std::cout << "\n\n Time taken for execution : " << elapsed_time << std::endl;

}