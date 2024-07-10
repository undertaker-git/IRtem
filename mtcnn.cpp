#include <mtcnn.h>

float timeSub = 0;
bool speedUp = false;
Rect2d bbox;
Point previousPoint, currentPoint;
Mat firstFrame;
vector<Point> predictP(4);
int MtcnnDetectionInterval = 15;
extern int MtcnnFrameCount;
std::vector<Ptr<Tracker>> Mtcnntrackers;


const int statuNum = 4;
const int measureNum = 2;

RNG MtcnnRng;
KalmanFilter MtcnnKF(statuNum, measureNum, 0);

//MtcnnKF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
//setIdentity(MtcnnKF.measurementMatrix);                                             //测量矩阵H
//setIdentity(MtcnnKF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
//setIdentity(MtcnnKF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
//setIdentity(MtcnnKF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
//MtcnnRng.fill(MtcnnKF.statePost,RNG::UNIFORM,0,640);                                //初始状态值x(0)

Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
vector<KalmanFilter> KFs(4,MtcnnKF);


class mtcnn{
public:
    mtcnn();
    void detect(ncnn::Mat& img_, std::vector<Bbox>& finalBbox);
private:
    void generateBbox(ncnn::Mat score, ncnn::Mat location, vector<Bbox>& boundingBox_, vector<orderScore>& bboxScore_, float scale);
    void nms(vector<Bbox> &boundingBox_, std::vector<orderScore> &bboxScore_, const float overlap_threshold, string modelname="Union");
    void refineAndSquareBbox(vector<Bbox> &vecBbox, const int &height, const int &width);

    ncnn::Net Pnet, Rnet, Onet;
    ncnn::Mat img;

    const float nms_threshold[3] = {0.5, 0.7, 0.7};
    const float threshold[3] = {0.6, 0.6, 0.6};
    const float mean_vals[3] = {127.5, 127.5, 127.5};
    const float norm_vals[3] = {0.0078125, 0.0078125, 0.0078125};
    std::vector<Bbox> firstBbox_, secondBbox_,thirdBbox_;
    std::vector<orderScore> firstOrderScore_, secondBboxScore_, thirdBboxScore_;
    int img_w, img_h;
};

//绝对路径开机自动启动才可以启动
mtcnn::mtcnn(){
    Pnet.load_param("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det1.param");
    Pnet.load_model("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det1.bin");
    Rnet.load_param("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det2.param");
    Rnet.load_model("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det2.bin");
    Onet.load_param("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det3.param");
    Onet.load_model("/home/pi/lixicai/Lepton_image_V6/RaspberrypiVideo_Lepton/models/det3.bin");
}


//框选目标
void draw_rectangle(int event, int x, int y, int flags, void*)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        previousPoint = Point(x, y);
    }
    else if (event == EVENT_MOUSEMOVE && (flags&EVENT_FLAG_LBUTTON))
    {
        Mat tmp;
        firstFrame.copyTo(tmp);
        currentPoint = Point(x, y);
        rectangle(tmp, previousPoint, currentPoint, Scalar(0, 255, 0, 0), 1, 8, 0);
        imshow("output", tmp);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        bbox.x = previousPoint.x;
        bbox.y = previousPoint.y;
        bbox.width = abs(previousPoint.x-currentPoint.x);
        bbox.height =  abs(previousPoint.y-currentPoint.y);
    }
    else if (event == EVENT_RBUTTONUP)
    {
        destroyWindow("output");
    }
}


bool cmpScore(orderScore lsh, orderScore rsh)
{
    if(lsh.score<rsh.score)
        return true;
    else
        return false;
}

static float getElapse(struct timeval *tv1,struct timeval *tv2)
{
    float t = 0.0f;
    if (tv1->tv_sec == tv2->tv_sec)
        t = (tv2->tv_usec - tv1->tv_usec)/1000.0f;
    else
        t = ((tv2->tv_sec - tv1->tv_sec) * 1000 * 1000 + tv2->tv_usec - tv1->tv_usec)/1000.0f;
    return t;
}


void mtcnn::generateBbox(ncnn::Mat score, ncnn::Mat location, std::vector<Bbox>& boundingBox_, std::vector<orderScore>& bboxScore_, float scale){
    int stride = 2;
    int cellsize = 12;
    int count = 0;
    //score p
    float *p = score.channel(1);
    float *plocal = location.channel(0);
    Bbox bbox;
    orderScore order;
    for(int row=0;row<score.h;row++){
        for(int col=0;col<score.w;col++){
            if(*p>threshold[0]){
                bbox.score = *p;
                order.score = *p;
                order.oriOrder = count;
                bbox.x1 = round((stride*col+1)/scale);
                bbox.y1 = round((stride*row+1)/scale);
                bbox.x2 = round((stride*col+1+cellsize)/scale);
                bbox.y2 = round((stride*row+1+cellsize)/scale);
                bbox.exist = true;
                bbox.area = (bbox.x2 - bbox.x1)*(bbox.y2 - bbox.y1);
                for(int channel=0;channel<4;channel++)
                    bbox.regreCoord[channel]=location.channel(channel)[0];
                boundingBox_.push_back(bbox);
                bboxScore_.push_back(order);
                count++;
            }
            p++;
            plocal++;
        }
    }
}

void mtcnn::nms(std::vector<Bbox> &boundingBox_, std::vector<orderScore> &bboxScore_, const float overlap_threshold, string modelname){
    if(boundingBox_.empty()){
        return;
    }
    std::vector<int> heros;
    //sort the score
    sort(bboxScore_.begin(), bboxScore_.end(), cmpScore);

    int order = 0;
    float IOU = 0;
    float maxX = 0;
    float maxY = 0;
    float minX = 0;
    float minY = 0;
    while(bboxScore_.size()>0){
        order = bboxScore_.back().oriOrder;
        bboxScore_.pop_back();
        if(order<0)continue;
        if(boundingBox_.at(order).exist == false) continue;
        heros.push_back(order);
        boundingBox_.at(order).exist = false;//delete it

        for(int num=0;num<boundingBox_.size();num++){
            if(boundingBox_.at(num).exist){
                //the iou
                maxX = (boundingBox_.at(num).x1>boundingBox_.at(order).x1)?boundingBox_.at(num).x1:boundingBox_.at(order).x1;
                maxY = (boundingBox_.at(num).y1>boundingBox_.at(order).y1)?boundingBox_.at(num).y1:boundingBox_.at(order).y1;
                minX = (boundingBox_.at(num).x2<boundingBox_.at(order).x2)?boundingBox_.at(num).x2:boundingBox_.at(order).x2;
                minY = (boundingBox_.at(num).y2<boundingBox_.at(order).y2)?boundingBox_.at(num).y2:boundingBox_.at(order).y2;
                //maxX1 and maxY1 reuse
                maxX = ((minX-maxX+1)>0)?(minX-maxX+1):0;
                maxY = ((minY-maxY+1)>0)?(minY-maxY+1):0;
                //IOU reuse for the area of two bbox
                IOU = maxX * maxY;
                if(!modelname.compare("Union"))
                    IOU = IOU/(boundingBox_.at(num).area + boundingBox_.at(order).area - IOU);
                else if(!modelname.compare("Min")){
                    IOU = IOU/((boundingBox_.at(num).area<boundingBox_.at(order).area)?boundingBox_.at(num).area:boundingBox_.at(order).area);
                }
                if(IOU>overlap_threshold){
                    boundingBox_.at(num).exist=false;
                    for(vector<orderScore>::iterator it=bboxScore_.begin(); it!=bboxScore_.end();it++){
                        if((*it).oriOrder == num) {
                            (*it).oriOrder = -1;
                            break;
                        }
                    }
                }
            }
        }
    }
    for(int i=0;i<heros.size();i++)
        boundingBox_.at(heros.at(i)).exist = true;
}


void mtcnn::refineAndSquareBbox(vector<Bbox> &vecBbox, const int &height, const int &width){
    if(vecBbox.empty()){
        cout<<"Bbox is empty!!"<<endl;
        return;
    }
    float bbw=0, bbh=0, maxSide=0;
    float h = 0, w = 0;
    float x1=0, y1=0, x2=0, y2=0;
    for(vector<Bbox>::iterator it=vecBbox.begin(); it!=vecBbox.end();it++){
        if((*it).exist){
            bbw = (*it).x2 - (*it).x1 + 1;
            bbh = (*it).y2 - (*it).y1 + 1;
            x1 = (*it).x1 + (*it).regreCoord[0]*bbw;
            y1 = (*it).y1 + (*it).regreCoord[1]*bbh;
            x2 = (*it).x2 + (*it).regreCoord[2]*bbw;
            y2 = (*it).y2 + (*it).regreCoord[3]*bbh;

            w = x2 - x1 + 1;
            h = y2 - y1 + 1;

            maxSide = (h>w)?h:w;
            x1 = x1 + w*0.5 - maxSide*0.5;
            y1 = y1 + h*0.5 - maxSide*0.5;
            (*it).x2 = round(x1 + maxSide - 1);
            (*it).y2 = round(y1 + maxSide - 1);
            (*it).x1 = round(x1);
            (*it).y1 = round(y1);

            //boundary check
            if((*it).x1<0)(*it).x1=0;
            if((*it).y1<0)(*it).y1=0;
            if((*it).x2>width)(*it).x2 = width - 1;
            if((*it).y2>height)(*it).y2 = height - 1;

            it->area = (it->x2 - it->x1)*(it->y2 - it->y1);
        }
    }
}

void mtcnn::detect(ncnn::Mat& img_, std::vector<Bbox>& finalBbox_){
    firstBbox_.clear();
    firstOrderScore_.clear();
    secondBbox_.clear();
    secondBboxScore_.clear();
    thirdBbox_.clear();
    thirdBboxScore_.clear();

    img = img_;
    img_w = img.w;
    img_h = img.h;
    img.substract_mean_normalize(mean_vals, norm_vals);

    float minl = img_w<img_h?img_w:img_h;
    int MIN_DET_SIZE = 12;
    int minsize = 50; //90
    float m = (float)MIN_DET_SIZE/minsize;
    minl *= m;
    float factor = 0.709;//0.709
    int factor_count = 0;
    vector<float> scales_;
    while(minl>MIN_DET_SIZE){
        if(factor_count>0)m = m*factor;
        scales_.push_back(m);
        minl *= factor;
        factor_count++;
    }
    orderScore order;
    int count = 0;

    struct timeval  tvP1,tvP2;
    struct timezone tzP1,tzP2;
    gettimeofday(&tvP1,&tzP1);
    for (size_t i = 0; i < scales_.size(); i++) {
        int hs = (int)ceil(img_h*scales_[i]);
        int ws = (int)ceil(img_w*scales_[i]);
        //ncnn::Mat in = ncnn::Mat::from_pixels_resize(image_data, ncnn::Mat::PIXEL_RGB2BGR, img_w, img_h, ws, hs);
        ncnn::Mat in;
        resize_bilinear(img_, in, ws, hs);
        //in.substract_mean_normalize(mean_vals, norm_vals);
        ncnn::Extractor ex = Pnet.create_extractor();
        ex.set_light_mode(true);
        ex.set_num_threads(4);
        ex.input("data", in);
        ncnn::Mat score_, location_;
        ex.extract("prob1", score_);
        ex.extract("conv4-2", location_);
        std::vector<Bbox> boundingBox_;
        std::vector<orderScore> bboxScore_;
        generateBbox(score_, location_, boundingBox_, bboxScore_, scales_[i]);
        nms(boundingBox_, bboxScore_, nms_threshold[0]);

        for(vector<Bbox>::iterator it=boundingBox_.begin(); it!=boundingBox_.end();it++){
            if((*it).exist){
                firstBbox_.push_back(*it);
                order.score = (*it).score;
                order.oriOrder = count;
                firstOrderScore_.push_back(order);
                count++;
            }
        }
        bboxScore_.clear();
        boundingBox_.clear();
    }
    //the first stage's nms
    if(count<1)return;
    nms(firstBbox_, firstOrderScore_, nms_threshold[0]);
    refineAndSquareBbox(firstBbox_, img_h, img_w);
    //printf("firstBbox_.size()=%d\n", firstBbox_.size());
    gettimeofday(&tvP2,&tzP2);
    //printf( "%s = %g ms \n ", "Pnet time", getElapse(&tvP1, &tvP2) );

    //second stage
    struct timeval  tvR1,tvR2;
    struct timezone tzR1,tzR2;
    //gettimeofday(&tvR1,&tzR1);
    count = 0;
    if(speedUp){
        timeSub = getElapse(&tvP1, &tvP2);
        struct timeval  tvR1,tvR2;
        struct timezone tzR1,tzR2;
        gettimeofday(&tvR1,&tzR1);
        for(vector<Bbox>::iterator it=firstBbox_.begin(); it!=firstBbox_.end();it++){
            int w = it->x2 - it->x1;
            int h = it->y2 - it->y1;
            //cout <<endl<< Point(it->x1, it->y1) << Point(it->x2,it->y2);
            bool isIn = false;
            for(vector<Point>::iterator p = predictP.begin();p!=predictP.end();++p){
                if((p->x+w/2)>(it->x1) && (p->x-w/2)<(it->x2) && (p->y+h/2)>(it->y1) && (p->y-h/2)<(it->y2) ){
                    isIn = true;
                    break;
                }
            }
            if(!isIn){
                it->exist = false;
            }
        }
        gettimeofday(&tvR2,&tzR2);
        timeSub += getElapse(&tvR1, &tvR2);
        for(vector<Bbox>::iterator it=firstBbox_.begin(); it!=firstBbox_.end();it++){
            if((*it).exist){
                cout << "YES!!!!";
                ncnn::Mat tempIm;
                copy_cut_border(img, tempIm, (*it).y1, img_h-(*it).y2, (*it).x1, img_w-(*it).x2);
                ncnn::Mat in;
                resize_bilinear(tempIm, in, 24, 24);
                ncnn::Extractor ex = Rnet.create_extractor();
                ex.set_light_mode(true);
                ex.set_num_threads(4);
                ex.input("data", in);
                ncnn::Mat score, bbox;
                ex.extract("prob1", score);
                ex.extract("conv5-2", bbox);
                if((score[1])>threshold[1]){
                    for(int channel=0;channel<4;channel++)
                        it->regreCoord[channel]=bbox[channel];
                    it->area = (it->x2 - it->x1)*(it->y2 - it->y1);
                    it->score = score[1];
                    secondBbox_.push_back(*it);
                    order.score = it->score;
                    order.oriOrder = count++;
                    secondBboxScore_.push_back(order);
                }
                else{
                    (*it).exist=false;
                }
            }
        }
    }
    else{
        timeSub = 0;
        for(vector<Bbox>::iterator it=firstBbox_.begin(); it!=firstBbox_.end();it++){
            //cout <<endl<< Point(it->x1, it->y1) << Point(it->x2,it->y2);
            if((*it).exist){
                ncnn::Mat tempIm;
                copy_cut_border(img, tempIm, (*it).y1, img_h-(*it).y2, (*it).x1, img_w-(*it).x2);
                ncnn::Mat in;
                resize_bilinear(tempIm, in, 24, 24);
                ncnn::Extractor ex = Rnet.create_extractor();
                ex.set_light_mode(true);
                ex.set_num_threads(4);
                ex.input("data", in);
                ncnn::Mat score, bbox;
                ex.extract("prob1", score);
                ex.extract("conv5-2", bbox);
                if((score[1])>threshold[1]){
                    for(int channel=0;channel<4;channel++)
                        it->regreCoord[channel]=bbox[channel];
                    it->area = (it->x2 - it->x1)*(it->y2 - it->y1);
                    it->score = score[1];
                    secondBbox_.push_back(*it);
                    order.score = it->score;
                    order.oriOrder = count++;
                    secondBboxScore_.push_back(order);
                }
                else{
                    (*it).exist=false;
                }
            }
        }
    }

    //printf("secondBbox_.size()=%d\n", secondBbox_.size());
    if(count<1)return;
    nms(secondBbox_, secondBboxScore_, nms_threshold[1]);
    refineAndSquareBbox(secondBbox_, img_h, img_w);
    //gettimeofday(&tvR2,&tzR2);
    //printf( "%s = %g ms \n ", "Rnet time", getElapse(&tvR1, &tvR2) );


    //third stage
    struct timeval  tvO1,tvO2;
    struct timezone tzO1,tzO2;
    //gettimeofday(&tvO1,&tzO1);
    count = 0;
    for(vector<Bbox>::iterator it=secondBbox_.begin(); it!=secondBbox_.end();it++){
        if((*it).exist){
            ncnn::Mat tempIm;
            copy_cut_border(img, tempIm, (*it).y1, img_h-(*it).y2, (*it).x1, img_w-(*it).x2);
            ncnn::Mat in;
            resize_bilinear(tempIm, in, 48, 48);
            ncnn::Extractor ex = Onet.create_extractor();
            ex.set_light_mode(true);
            ex.set_num_threads(4);
            ex.input("data", in);
            ncnn::Mat score, bbox, keyPoint;
            ex.extract("prob1", score);
            ex.extract("conv6-2", bbox);
            //ex.extract("conv6-3", keyPoint);
            if(score[1]>threshold[2]){
                for(int channel=0;channel<4;channel++)
                    it->regreCoord[channel]=bbox[channel];
                it->area = (it->x2 - it->x1)*(it->y2 - it->y1);
                it->score = score[1];
                //for(int num=0;num<5;num++){
                //    (it->ppoint)[num] = it->x1 + (it->x2 - it->x1)*keyPoint[num];
                //    (it->ppoint)[num+5] = it->y1 + (it->y2 - it->y1)*keyPoint[num+5];
                //}

                thirdBbox_.push_back(*it);
                order.score = it->score;
                order.oriOrder = count++;
                thirdBboxScore_.push_back(order);
            }
            else
                (*it).exist=false;
        }
    }

    //printf("thirdBbox_.size()=%d\n", thirdBbox_.size());
    if(count<1)return;
    refineAndSquareBbox(thirdBbox_, img_h, img_w);
    nms(thirdBbox_, thirdBboxScore_, nms_threshold[2], "Min");
    finalBbox_ = thirdBbox_;

    //gettimeofday(&tvO2,&tzO2);
    //printf( "%s = %g ms \n ", "Onet time", getElapse(&tvO1, &tvO2) );
}



void MtcnnDetection(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y , Rect2d roibox)
{
    if (MtcnnFrameCount % MtcnnDetectionInterval == 0)
    {
        int bboxId = 0;
        for(vector<Point>::iterator it = predictP.begin();it!=predictP.end();++it){
            Mat prediction = KFs[bboxId].predict();
            bboxId++;
            *it = Point(prediction.at<float>(0),prediction.at<float>(1));
            cout <<MtcnnFrameCount << endl << "predict center: "<<*it;
        }
        if(MtcnnFrameCount >= KALMAN_BEGIN){
            speedUp = true;
        }
        if((MtcnnFrameCount-KALMAN_BEGIN)%KALMAN_INTERVAL==0){
            speedUp = false;
        }
        // detect and reset all trackers
        std::vector<Bbox> finalBbox;
        mtcnn mm;
        ncnn::Mat ncnn_img = ncnn::Mat::from_pixels(ImgDetect.data, ncnn::Mat::PIXEL_BGR2RGB, ImgDetect.cols, ImgDetect.rows);
        struct timeval  tv1,tv2;
        struct timezone tz1,tz2;

        gettimeofday(&tv1,&tz1);
        mm.detect(ncnn_img, finalBbox);
        gettimeofday(&tv2,&tz2);

        //reset all trackers
        Mtcnntrackers.clear();
        bboxId = 0;
        //printf( "%s = %g ms \n ", "Detection All time", getElapse(&tv1, &tv2) - timeSub);
        for(vector<Bbox>::iterator it=finalBbox.begin(); it!=finalBbox.end(); it++)
        {
            if((*it).exist)
            {
                // init tracker
                Ptr<Tracker> tracker = TrackerMOSSE::create();
                tracker->init(ImgShow, Rect2d(Point((*it).x1, (*it).y1), Point((*it).x2, (*it).y2)));
                Mtcnntrackers.push_back(tracker);

                measurement.at<float>(0) = (float)((*it).x1+(*it).x2)/2;
                measurement.at<float>(1) = (float)((*it).y1+(*it).y2)/2;
                KFs[bboxId].correct(measurement);
                bboxId++;

                rectangle(ImgShow, Point(offset_x+(*it).x1, offset_y+(*it).y1), Point(offset_x+(*it).x2, offset_y+(*it).y2), Scalar(0,0,255), 2,8,0);
                cout << endl << "predict center: "<< Point2f((float)((*it).x1+(*it).x2)/2, (float)((*it).y1+(*it).y2)/2);
                // for(int num=0;num<5;num++)circle(cv_img,Point((int)*(it->ppoint+num), (int)*(it->ppoint+num+5)),3,Scalar(0,255,255), -1);
            }
        }
    }
    else
    {
        // just track
        for (int i=0; i<Mtcnntrackers.size(); i++)
        {
            struct timeval  tv1,tv2;
            struct timezone tz1,tz2;

            Ptr<Tracker> tracker = Mtcnntrackers[i];
            Rect2d bbox(0, 0, 0, 0);

            gettimeofday(&tv1,&tz1);
            if (tracker->update(ImgShow, bbox))
            {
                //rectangle(ImgShow, bbox.tl(), bbox.br(), Scalar(0,0,255), 2,8,0);
                rectangle(ImgShow, Point(offset_x+bbox.tl().x, offset_y+bbox.tl().y), Point(offset_x+bbox.br().x, offset_y+bbox.br().y), Scalar(0,0,255), 2,8,0);
                gettimeofday(&tv2,&tz2);
                //printf( "%s = %g ms \n ", "Track All time", getElapse(&tv1, &tv2) );
            }
        }
     }
     //MtcnnFrameCount++;
}



void FaceDetect(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y,Rect2d roibox)
{
    //rectangle(ImgShow, Point(roibox.x,roibox.y), Point(roibox.x+roibox.width, roibox.y+roibox.height), Scalar(255,0,0), 2,8,0);
    if (MtcnnFrameCount % MtcnnDetectionInterval == 0)
    {
        // detect and reset all trackers
        MtcnnFrameCount=0;
        std::vector<Bbox> finalBbox;
        mtcnn mm;
        ncnn::Mat ncnn_img = ncnn::Mat::from_pixels(ImgDetect.data, ncnn::Mat::PIXEL_BGR2RGB, ImgDetect.cols, ImgDetect.rows);
        struct timeval  tv1,tv2;
        struct timezone tz1,tz2;

        gettimeofday(&tv1,&tz1);
        mm.detect(ncnn_img, finalBbox);
        gettimeofday(&tv2,&tz2);

        //reset all trackers
        Mtcnntrackers.clear();

        //printf( "%s = %g ms \n ", "Detection All time", getElapse(&tv1, &tv2) );
        for(vector<Bbox>::iterator it=finalBbox.begin(); it!=finalBbox.end();it++)
        {
            if((*it).exist)
            {
                // init tracker
                Ptr<Tracker> tracker = TrackerMOSSE::create();
                tracker->init(ImgShow, Rect2d(Point((*it).x1+roibox.x, (*it).y1+roibox.y), Point((*it).x2+roibox.x, (*it).y2+roibox.y)));
                Mtcnntrackers.push_back(tracker);

                //rectangle(ImgShow, Point((*it).x1, (*it).y1), Point((*it).x2, (*it).y2), Scalar(0,0,255), 2,8,0);
                rectangle(ImgShow, Point((*it).x1+roibox.x, (*it).y1+roibox.y), Point((*it).x2+roibox.x, (*it).y2+roibox.y), Scalar(0,0,255), 2,8,0);
                //for(int num=0;num<5;num++)circle(cv_img,Point((int)*(it->ppoint+num), (int)*(it->ppoint+num+5)),3,Scalar(0,255,255), -1);
            }
        }
    }
    else
    {
        // just track
        for (int i=0; i<Mtcnntrackers.size(); i++)
        {
            //struct timeval  tv1,tv2;
            //struct timezone tz1,tz2;

            Ptr<Tracker> tracker = Mtcnntrackers[i];
            Rect2d bbox(0, 0, 0, 0);

            //gettimeofday(&tv1,&tz1);
            if (tracker->update(ImgShow, bbox))
            {
                //rectangle(ImgShow, Point(offset_x+bbox.tl().x, offset_y+bbox.tl().y), Point(offset_x+bbox.br().x, offset_y+bbox.br().y), Scalar(0,0,255), 2,8,0);
                rectangle(ImgShow, bbox.tl(), bbox.br(), Scalar(0,0,255), 2,8,0);
                //gettimeofday(&tv2,&tz2);
                //printf( "%s = %g ms \n ", "Track All time", getElapse(&tv1, &tv2) );
            }
        }
     }
    //MtcnnFrameCount++;
}




void MtcnnFaceDetect_ROI(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y,Rect2d roibox ,int RoiBoxNum)
{
    //rectangle(ImgShow, Point(roibox.x,roibox.y), Point(roibox.x+roibox.width, roibox.y+roibox.height), Scalar(255,0,0), 2,8,0);
    if (MtcnnFrameCount % MtcnnDetectionInterval == 0)
    {
        // detect and reset all trackers
        std::vector<Bbox> finalBbox;
        mtcnn mm;
        ncnn::Mat ncnn_img = ncnn::Mat::from_pixels(ImgDetect.data, ncnn::Mat::PIXEL_BGR2RGB, ImgDetect.cols, ImgDetect.rows);
        struct timeval  tv1,tv2;
        struct timezone tz1,tz2;

        gettimeofday(&tv1,&tz1);
        mm.detect(ncnn_img, finalBbox);
        vector<Bbox>::iterator it=finalBbox.begin();
        gettimeofday(&tv2,&tz2);

        //reset all trackers
        Mtcnntrackers.clear();
        MtcnnFrameCount=0;

        //printf( "%s = %g ms \n ", "Detection All time", getElapse(&tv1, &tv2) );
        for(int cnt =0; cnt <= RoiBoxNum ;cnt++)
        {
             if((*it).exist)
             {
                 // init tracker
                 Ptr<Tracker> tracker = TrackerMOSSE::create();
                 tracker->init(ImgShow, Rect2d(Point((*it).x1+roibox.x, (*it).y1+roibox.y), Point((*it).x2+roibox.x, (*it).y2+roibox.y)));
                 Mtcnntrackers.push_back(tracker);
                 rectangle(ImgShow, Point((*it).x1+roibox.x, (*it).y1+roibox.y), Point((*it).x2+roibox.x, (*it).y2+roibox.y), Scalar(0,0,255), 2,8,0);
                 //for(int num=0;num<5;num++)circle(ImgShow,Point((int)*(it->ppoint+num), (int)*(it->ppoint+num+5)),3,Scalar(0,255,255), -1);
             }
        }
    }
    else
    {
        // just track
        for (int i=0; i<Mtcnntrackers.size(); i++)
        {
            Ptr<Tracker> tracker = Mtcnntrackers[i];
            Rect2d bbox(0, 0, 0, 0);

            if (tracker->update(ImgShow, bbox))
            {
                rectangle(ImgShow, bbox.tl(), bbox.br(), Scalar(0,0,255), 2,8,0);
            }
        }
     }
}




void MtcnnFaceDetect(Mat ImgDetect,Mat ImgShow)
{
    //rectangle(ImgShow, Point(roibox.x,roibox.y), Point(roibox.x+roibox.width, roibox.y+roibox.height), Scalar(255,0,0), 2,8,0);
    if (MtcnnFrameCount % MtcnnDetectionInterval == 0)
    {
        // detect and reset all trackers
        MtcnnFrameCount=0;
        std::vector<Bbox> finalBbox;
        mtcnn mm;
        ncnn::Mat ncnn_img = ncnn::Mat::from_pixels(ImgDetect.data, ncnn::Mat::PIXEL_BGR2RGB, ImgDetect.cols, ImgDetect.rows);
        struct timeval  tv1,tv2;
        struct timezone tz1,tz2;

        gettimeofday(&tv1,&tz1);
        mm.detect(ncnn_img, finalBbox);
        gettimeofday(&tv2,&tz2);

        //reset all trackers
        Mtcnntrackers.clear();

        //printf( "%s = %g ms \n ", "Detection All time", getElapse(&tv1, &tv2) );
        for(vector<Bbox>::iterator it=finalBbox.begin(); it!=finalBbox.end();it++)
        {
            if((*it).exist)
            {
                // init tracker
                Ptr<Tracker> tracker = TrackerMOSSE::create();
                tracker->init(ImgShow, Rect2d(Point((*it).x1, (*it).y1), Point((*it).x2, (*it).y2)));
                Mtcnntrackers.push_back(tracker);
                rectangle(ImgShow, Point((*it).x1, (*it).y1), Point((*it).x2, (*it).y2), Scalar(0,0,255), 2,8,0);
                //for(int num=0;num<5;num++)circle(ImgShow,Point((int)*(it->ppoint+num), (int)*(it->ppoint+num+5)),3,Scalar(0,255,255), -1);
            }
        }
    }
    else
    {
        // just track
        for (int i=0; i<Mtcnntrackers.size(); i++)
        {
            Ptr<Tracker> tracker = Mtcnntrackers[i];
            Rect2d bbox(0, 0, 0, 0);

            if (tracker->update(ImgShow, bbox))
            {
                rectangle(ImgShow, bbox.tl(), bbox.br(), Scalar(0,0,255), 2,8,0);
            }
        }
     }
}

