/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#ifndef _Aruco_FAST_h
#define _Aruco_FAST_h



#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

namespace aruco
{
    /**
     * FAST CORNER DETECTION METHOD
     */
    class   FAST
    {
    public:
        inline FAST();
        inline void setParams(const cv::Mat &image, int thres);

        inline cv::Point getMaxInRegion(cv::Rect rect);
        inline cv::Point getMinInRegion(cv::Rect rect);
        inline cv::Point2f getAvrgInRegion(cv::Rect rect);
    private:
        inline void makeOffsets(int pixel[25], int rowStride, int patternSize);
        inline int  cornerScore(const uchar* ptr, const int pixel[], int threshold);


        cv::Mat _image;
        cv::Mat _visited;
        uchar threshold_tab[512];
        int threshold=7;
        int pixel[25];
        const int patternSize=16;
    };

inline FAST::FAST(){

}

inline cv::Point2f FAST::getAvrgInRegion(cv::Rect rect){
    int endy=rect.y+rect.height;
    int endx=rect.x+rect.width;
    if (rect.x<3) rect.x=3;
    if (rect.y<3) rect.y=3;
    if ( endx>=_image.cols-3) endx=_image.cols-3;
    if ( endy>=_image.rows-3) endy=_image.rows-3;
    cv::Point2d avg(0,0);
    double sumscore=0;
    for(int y=rect.y;y<endy; y++)
    {
        const auto img_ptr=_image.ptr<uchar>(y);
        for(int x=rect.x;x<endx; x++){
            double score= (uchar)cornerScore (img_ptr+x, pixel, threshold);
            score=score*score*score;
            sumscore+=score;
            avg.x+=x*score;
            avg.y+=y*score;
        }
    }

    if ( sumscore!=0)
       return  cv::Point2f( float(avg.x)/float(sumscore), float(avg.y)/float(sumscore));
    else return cv::Point2f(rect.x+rect.width/2.f,rect.y+rect.height/2.f);
}
inline cv::Point FAST::getMaxInRegion(cv::Rect rect){
    int endy=rect.y+rect.height;
    int endx=rect.x+rect.width;
    if (rect.x<3) rect.x=3;
    if (rect.y<3) rect.y=3;
    if ( endx>=_image.cols-3) endx=_image.cols-3;
    if ( endy>=_image.rows-3) endy=_image.rows-3;
    std::pair<cv::Point,uint16_t> maxVal ( cv::Point(-1,-1),std::numeric_limits<uint16_t>::lowest());
    for(int y=rect.y;y<endy; y++)
    {
        auto ptr=_visited.ptr<uint16_t>(y);
        const auto img_ptr=_image.ptr<uchar>(y);
        for(int x=rect.x;x<endx; x++){
            if (ptr[x]==std::numeric_limits<uint16_t>::max())
                ptr[x]= (uchar)cornerScore (img_ptr+x, pixel, threshold);
            if ( maxVal.second<ptr[x])
                maxVal ={cv::Point(x,y),ptr[x]};
        }
    }
    return maxVal.first;
}
inline cv::Point FAST::getMinInRegion(cv::Rect rect){
    int endy=rect.y+rect.height;
    int endx=rect.x+rect.width;
    if (rect.x<3) rect.x=3;
    if (rect.y<3) rect.y=3;
    if ( endx>=_image.cols-3) endx=_image.cols-3;
    if ( endy>=_image.rows-3) endy=_image.rows-3;
    std::pair<cv::Point,uint16_t> minVal ( cv::Point(-1,-1),std::numeric_limits<uint16_t>::max());
    for(int y=rect.y;y<endy; y++)
    {
        auto ptr=_visited.ptr<uint16_t>(y);
        const auto img_ptr=_image.ptr<uchar>(y);
        for(int x=rect.x;x<endx; x++){
            if (ptr[x]==std::numeric_limits<uint16_t>::max())
                ptr[x]= (uchar)cornerScore (img_ptr+x, pixel, threshold);
            if ( minVal.second>ptr[x])
                minVal ={cv::Point(x,y),ptr[x]};
        }
    }
    return minVal.first;
}

inline void FAST::setParams(const cv::Mat &image,int thres){
    assert(_image.type()==CV_8UC1);
    bool bmakeOffsets=false;
    if ( _image.empty())bmakeOffsets=true;
    else if (_image.step!=image.step)bmakeOffsets=true;
    if (bmakeOffsets) makeOffsets(pixel, (int)image.step, patternSize);
    threshold = std::min(std::max(thres, 0), 255);
    _image=image;
     for(int i = -255; i <= 255; i++ )
        threshold_tab[i+255] = (uchar)(i < -threshold ? 1 : i > threshold ? 2 : 0);

}



inline void FAST::makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int (*offsets)[2] = patternSize == 16 ? offsets16 :
                              patternSize == 12 ? offsets12 :
                              patternSize == 8  ? offsets8  : 0;

    CV_Assert(pixel && offsets);

    int k = 0;
    for( ; k < patternSize; k++ )
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for( ; k < 25; k++ )
        pixel[k] = pixel[k - patternSize];
}
inline int FAST::cornerScore(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

    int a0 = threshold;
    for( k = 0; k < 16; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        a = std::min(a, (int)d[k+3]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+4]);
        a = std::min(a, (int)d[k+5]);
        a = std::min(a, (int)d[k+6]);
        a = std::min(a, (int)d[k+7]);
        a = std::min(a, (int)d[k+8]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+9]));
    }

    int b0 = -a0;
    for( k = 0; k < 16; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        b = std::max(b, (int)d[k+4]);
        b = std::max(b, (int)d[k+5]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+6]);
        b = std::max(b, (int)d[k+7]);
        b = std::max(b, (int)d[k+8]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+9]));
    }

    threshold = -b0-1;

#if VERIFY_CORNERS
    testCorner(ptr, pixel, K, N, threshold);
#endif
    return threshold;
}

}
#endif
