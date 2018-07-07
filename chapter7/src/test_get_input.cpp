#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

void on_mouse(int event, int x, int y, int flags, void* param) {
    switch (event) {
    case CV_EVENT_MOUSEMOVE:
        DisplayImage.at<cv::Vec3b>(y, x).val[1] = 255;
).
val[2] = 255;

DisplayImage.at<cv:
:Vec3b>(y , x

break;
case CV_EVENT_LBUTTONDOWN:
v
::
Point(x , y
) , 5 , c
v
::
Scalar(255 , 255 , 255) , 2
);


cv:
:circle(DisplayImage , c

break;
case CV_EVENT_RBUTTONDOWN:
) , cv: :
Point(x+20 , y
) ,

cv: :li 且 e(DisplayImage , cv: :Point(x-20 , y
c
v
: :
Scalar(255 , 255 , 0));
)
;

cv::Scalar(255 , 255 , 0

break;
case CV EVENT RBUTTONUP:

c
v
::
line(DisplayImage , cv:
:Point(x , y-20) , cv::Point(x , y+20) ,
cv:
:Scalar(255 , 0 , 255));

c
v
::
Scalar(255 , 0 , 255);

break;
default:

break;
}
display" , DisplayImage);
c
v
: :
imshow(' ・
}

む * 釘 g
v
[
]
){
int main(int argc , ch
DisplayImage - cv::Mat::
zeros(300 , 300 , CV_8UC3);
cv: ・ imshow("display' DisplayImage);
)
;
cvSetMouseCallback("display" , on_mouse , 0
while(l)

if (cv:
:waitKey(O) =
= 'q')

break;

return;

}
