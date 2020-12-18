 #include <iostream>
 #include <opencv2/opencv_modules.hpp>
 
 #include <opencv2/line_descriptor.hpp>
 #include <opencv2/core/utility.hpp>
 #include <opencv2/imgproc.hpp>
 #include <opencv2/features2d.hpp>
 #include <opencv2/highgui.hpp>
 #include "liof/liof.h"

//#define nfeatures 1000;
#define MATCHES_DIST_THRESHOLD 200

using namespace cv;
using namespace std;

static const char* keys =
 { "{@image_path1 | | Image path 1 }"
     "{@image_path2 | | Image path 2 }" };
 

//ratio test nearest/second nearest < ratio
vector<DMatch> ratio_test(vector< vector<DMatch> > matches12, double ratio){
    vector<DMatch> good_matches;         
    for(int i = 0; i < matches12.size(); i++){  
        if(matches12[i][0].distance < ratio * matches12[i][1].distance)
            good_matches.push_back(matches12[i][0]);
    }
    return good_matches;                  
}

// Symmetric Test...
vector<DMatch> symmetric_test(std::vector<DMatch> good_matches1, std::vector<DMatch> good_matches2){
    std::vector<DMatch> better_matches;
    for(int i=0; i<good_matches1.size(); i++){
        for(int j=0; j<good_matches2.size(); j++){
            if(good_matches1[i].queryIdx == good_matches2[j].trainIdx && good_matches2[j].queryIdx == good_matches1[i].trainIdx){
                better_matches.push_back(DMatch(good_matches1[i].queryIdx, good_matches1[i].trainIdx, good_matches1[i].distance));
                break;
            }
        }
    }

    return better_matches;
}

int main( int argc, char** argv )
{
    CommandLineParser parser( argc, argv, keys );
    String image_path1 = parser.get<String>( 0 );
    String image_path2 = parser.get<String>( 1 );
 
    if( image_path1.empty() || image_path2.empty() )
    {
        return -1;
    }
 
    /* load image */
    cv::Mat image = imread( image_path1, IMREAD_GRAYSCALE );
    cv::Mat image2 = imread( image_path2, IMREAD_GRAYSCALE );
    resize( image, image, Size( image.cols / 3, image.rows / 3 ), 0, 0, INTER_LINEAR_EXACT );
    resize( image2, image2, Size( image2.cols / 3, image2.rows / 3  ), 0, 0, INTER_LINEAR_EXACT );
    
    float b, c;
    b = -0.75;
    c = 1.5;
    Mat blur;
    GaussianBlur(image, blur, cv::Size(0, 0), 3);
    addWeighted(image, c, blur, b, 0, image);
    
    GaussianBlur(image2, blur, cv::Size(0, 0), 3);
    addWeighted(image2, c, blur, b, 0, image2);
    
    feature_detector::LIOF detector;
    
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1,descriptors2;

    vector< vector<DMatch> > matches12, matches21;

    #if(CV_VERSION_MAJOR == 4)
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::MatcherType::BRUTEFORCE_L1);
    #else
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(cv::NORM_HAMMING);
    #endif
  
    std::vector<string> result;

    clock_t begin = clock();
    detector.detect(image, keypoints1);
    
    //Mat int_img;
    //drawKeypoints(image, keypoints1, int_img);
    //imshow("image", int_img);
    //waitKey();
    detector.detect(image2, keypoints2);
    //cout << "Keypoints2 " << keypoints2.size() << endl;
    clock_t end_detect = clock();
    double elapsed_secs_detect = double(end_detect - begin) / CLOCKS_PER_SEC;
    cout << "elapsed secs detect " << elapsed_secs_detect << endl;
    // cout << "# keypoints of query_image :" << keypoints1.size() << endl;
    // cout << "# keypoints of image :" << keypoints2.size() << endl;

    //cout << "Compute Image1" << endl;
    detector.compute(image,keypoints1,descriptors1);
    //cout << "descriptors1 " << descriptors1.size() << endl;
    //cout << descriptors1 << endl;
    //return 1;
    //cout << "Compute Image2" << endl;
    detector.compute(image2,keypoints2,descriptors2);
    //cout << "Descriptors size :" << descriptors1.cols << ":"<< descriptors1.rows << endl;
    //cout << "descriptors2 " << descriptors2.size() << endl;
    //cout << descriptors2 << endl;
    //cout << "Matching" << endl;
    clock_t end_compute = clock();
    double elapsed_secs_compute = double(end_compute - end_detect) / CLOCKS_PER_SEC;
    cout << "elapsed secs compute " << elapsed_secs_compute << endl;
    

    //3.Match the descriptors in two directions...
    matcher->knnMatch( descriptors1, descriptors2, matches12, 2 );
    matcher->knnMatch( descriptors2, descriptors1, matches21, 2 );

    
    //BFMatcher bfmatcher(NORM_L2, true);
    //vector<DMatch> matches, good_matches;
    //bfmatcher.match(descriptors1, descriptors2, matches);

    //for ( int i = 0; i < (int) matches.size(); i++ )
    //{
    //    if( matches[i].distance < MATCHES_DIST_THRESHOLD )
    //        good_matches.push_back( matches[i] );
    //}
    // cout << "Matches1-2:" << matches12.size() << endl;
    // cout << "Matches2-1:" << matches21.size() << endl;

    //4. ratio test proposed by David Lowe paper = 0.8
    double ratio = 0.9;
    std::vector<DMatch> good_matches1, good_matches2;
    good_matches1 = ratio_test(matches12, ratio);
    good_matches2 = ratio_test(matches21, ratio);

    // cout << "Good matches1:" << good_matches1.size() << endl;
    // cout << "Good matches2:" << good_matches2.size() << endl;

    // Symmetric Test
    std::vector<DMatch> better_matches;
    better_matches = symmetric_test(good_matches1, good_matches2);


    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    Mat out;
    drawMatches(image, keypoints1, image2, keypoints2, better_matches, out);
    cout << better_matches.size() << endl;
    imshow("out result", out);
    waitKey();
    //cout << "Better matches:" << better_matches.size() << endl;

    //5. Compute the similarity of this image pair...
    //float jaccard = 1.0 * better_matches.size() / (keypoints1.size() + keypoints2.size() - better_matches.size());
    // MatchScore ms;
    // ms.query_image = query_name.substr(query_name.find_last_of("/") + 1);
    // ms.train_image = name;
    // ms.query_image_keypoints_size = keypoints1.size();
    // ms.train_image_keypoints_size = keypoints2.size();
    //ms.good_matches_size =  better_matches.size();
    //ms.jaccard = jaccard;
    
    //match_scores.push_back(ms);
    //cout << query_name << "-" << image_name<< ",jaccard:" << jaccard << endl;
           

    // We got all the matches and its scores, so sort them :) FIXME to better top-k
    //result = match_scores_sort(match_scores, k);

    cout << "Time Costs : " << elapsed_secs << endl;
    //return result;
}