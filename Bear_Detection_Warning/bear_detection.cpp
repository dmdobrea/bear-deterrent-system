#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include <torch/script.h>

#include <signal.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pthread.h>
#include <semaphore.h>

#include <curl/curl.h>

using torch::indexing::Slice;
using torch::indexing::None;

void* writeDB (void *arg);
void sig_handler_INT (int num);
cv::Scalar get_color(int class_id);
int writeIntToFirebase    (const char *url, int value);
int writeStringToFirebase (const char *url, const char *value);
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method);

float generate_scale(cv::Mat& image, const std::vector<int>& target_size) 
{
    int origin_w = image.cols;
    int origin_h = image.rows;

    int target_h = target_size[0];
    int target_w = target_size[1];

    float ratio_h = static_cast<float>(target_h) / static_cast<float>(origin_h);
    float ratio_w = static_cast<float>(target_w) / static_cast<float>(origin_w);
    float resize_scale = std::min(ratio_h, ratio_w);
    
    return resize_scale;
}


float letterbox(cv::Mat &input_image, cv::Mat &output_image, const std::vector<int> &target_size) {
    if (input_image.cols == target_size[1] && input_image.rows == target_size[0]) 
    {
        if (input_image.data == output_image.data) 
        {
            return 1.;
        } else 
			{
            output_image = input_image.clone();
            return 1.;
			}
    }

    float resize_scale = generate_scale(input_image, target_size);
    int new_shape_w = std::round(input_image.cols * resize_scale);
    int new_shape_h = std::round(input_image.rows * resize_scale);
    float padw = (target_size[1] - new_shape_w) / 2.;
    float padh = (target_size[0] - new_shape_h) / 2.;

    int top = std::round(padh - 0.1);
    int bottom = std::round(padh + 0.1);
    int left = std::round(padw - 0.1);
    int right = std::round(padw + 0.1);

    cv::resize(input_image, output_image,
               cv::Size(new_shape_w, new_shape_h),
               0, 0, cv::INTER_AREA);

    cv::copyMakeBorder(output_image, output_image, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114., 114., 114));
    return resize_scale;
}


torch::Tensor xyxy2xywh(const torch::Tensor& x) 
{
    auto y = torch::empty_like(x);
    y.index_put_({"...", 0}, (x.index({"...", 0}) + x.index({"...", 2})).div(2));
    y.index_put_({"...", 1}, (x.index({"...", 1}) + x.index({"...", 3})).div(2));
    y.index_put_({"...", 2},  x.index({"...", 2}) - x.index({"...", 0}));
    y.index_put_({"...", 3},  x.index({"...", 3}) - x.index({"...", 1}));
    return y;
}


torch::Tensor xywh2xyxy(const torch::Tensor& x) 
{
    auto y = torch::empty_like(x);
    auto dw = x.index({"...", 2}).div(2);
    auto dh = x.index({"...", 3}).div(2);
    y.index_put_({"...", 0}, x.index({"...", 0}) - dw);
    y.index_put_({"...", 1}, x.index({"...", 1}) - dh);
    y.index_put_({"...", 2}, x.index({"...", 0}) + dw);
    y.index_put_({"...", 3}, x.index({"...", 1}) + dh);
    return y;
}


// Reference: https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
torch::Tensor nms(const torch::Tensor& bboxes, const torch::Tensor& scores, float iou_threshold) 
{
    if (bboxes.numel() == 0)
        return torch::empty({0}, bboxes.options().dtype(torch::kLong));

    auto x1_t = bboxes.select(1, 0).contiguous();
    auto y1_t = bboxes.select(1, 1).contiguous();
    auto x2_t = bboxes.select(1, 2).contiguous();
    auto y2_t = bboxes.select(1, 3).contiguous();

    torch::Tensor areas_t = (x2_t - x1_t) * (y2_t - y1_t);

    auto order_t = std::get<1>(
        scores.sort(/*stable=*/true, /*dim=*/0, /* descending=*/true));

    auto ndets = bboxes.size(0);
    torch::Tensor suppressed_t = torch::zeros({ndets}, bboxes.options().dtype(torch::kByte));
    torch::Tensor keep_t = torch::zeros({ndets}, bboxes.options().dtype(torch::kLong));

    auto suppressed = suppressed_t.data_ptr<uint8_t>();
    auto keep = keep_t.data_ptr<int64_t>();
    auto order = order_t.data_ptr<int64_t>();
    auto x1 = x1_t.data_ptr<float>();
    auto y1 = y1_t.data_ptr<float>();
    auto x2 = x2_t.data_ptr<float>();
    auto y2 = y2_t.data_ptr<float>();
    auto areas = areas_t.data_ptr<float>();

    int64_t num_to_keep = 0;

    for (int64_t _i = 0; _i < ndets; _i++) 
		{
        auto i = order[_i];
        if (suppressed[i] == 1)
            continue;
            
        keep[num_to_keep++] = i;
        auto ix1 = x1[i];
        auto iy1 = y1[i];
        auto ix2 = x2[i];
        auto iy2 = y2[i];
        auto iarea = areas[i];

        for (int64_t _j = _i + 1; _j < ndets; _j++) 
        {
			auto j = order[_j];
			if (suppressed[j] == 1)
				continue;
            
			auto xx1 = std::max(ix1, x1[j]);
			auto yy1 = std::max(iy1, y1[j]);
			auto xx2 = std::min(ix2, x2[j]);
			auto yy2 = std::min(iy2, y2[j]);

			auto w = std::max(static_cast<float>(0), xx2 - xx1);
			auto h = std::max(static_cast<float>(0), yy2 - yy1);
			auto inter = w * h;
			auto ovr = inter / (iarea + areas[j] - inter);
			if (ovr > iou_threshold)
				suppressed[j] = 1;
        }
    }
    return keep_t.narrow(0, 0, num_to_keep);
}


torch::Tensor non_max_suppression(torch::Tensor& prediction, float conf_thres = 0.25, float iou_thres = 0.45, int max_det = 300) 
{
    auto bs = prediction.size(0);
    auto nc = prediction.size(1) - 4;
    auto nm = prediction.size(1) - nc - 4;
    auto mi = 4 + nc;
    auto xc = prediction.index({Slice(), Slice(4, mi)}).amax(1) > conf_thres;

    prediction = prediction.transpose(-1, -2);
    prediction.index_put_({"...", Slice({None, 4})}, xywh2xyxy(prediction.index({"...", Slice(None, 4)})));

    std::vector<torch::Tensor> output;
    for (int i = 0; i < bs; i++) 
    {
        output.push_back(torch::zeros({0, 6 + nm}, prediction.device()));
    }

    for (int xi = 0; xi < prediction.size(0); xi++) 
    {
        auto x = prediction[xi];
        x = x.index({xc[xi]});
        auto x_split = x.split({4, nc, nm}, 1);
        auto box = x_split[0], cls = x_split[1], mask = x_split[2];
        auto [conf, j] = cls.max(1, true);
        x = torch::cat({box, conf, j.toType(torch::kFloat), mask}, 1);
        x = x.index({conf.view(-1) > conf_thres});
        int n = x.size(0);
        if (!n) { continue; }

        // NMS
        auto c = x.index({Slice(), Slice{5, 6}}) * 7680;
        auto boxes = x.index({Slice(), Slice(None, 4)}) + c;
        auto scores = x.index({Slice(), 4});
        auto i = nms(boxes, scores, iou_thres);
        i = i.index({Slice(None, max_det)});
        output[xi] = x.index({i});
    }

    return torch::stack(output);
}


torch::Tensor clip_boxes(torch::Tensor& boxes, const std::vector<int>& shape) 
{
    boxes.index_put_({"...", 0}, boxes.index({"...", 0}).clamp(0, shape[1]));
    boxes.index_put_({"...", 1}, boxes.index({"...", 1}).clamp(0, shape[0]));
    boxes.index_put_({"...", 2}, boxes.index({"...", 2}).clamp(0, shape[1]));
    boxes.index_put_({"...", 3}, boxes.index({"...", 3}).clamp(0, shape[0]));
    return boxes;
}


torch::Tensor scale_boxes(const std::vector<int>& img1_shape, torch::Tensor& boxes, const std::vector<int>& img0_shape) 
{
    auto gain = (std::min)((float)img1_shape[0] / img0_shape[0], (float)img1_shape[1] / img0_shape[1]);
    auto pad0 = std::round((float)(img1_shape[1] - img0_shape[1] * gain) / 2. - 0.1);
    auto pad1 = std::round((float)(img1_shape[0] - img0_shape[0] * gain) / 2. - 0.1);

    boxes.index_put_({"...", 0}, boxes.index({"...", 0}) - pad0);
    boxes.index_put_({"...", 2}, boxes.index({"...", 2}) - pad0);
    boxes.index_put_({"...", 1}, boxes.index({"...", 1}) - pad1);
    boxes.index_put_({"...", 3}, boxes.index({"...", 3}) - pad1);
    boxes.index_put_({"...", Slice(None, 4)}, boxes.index({"...", Slice(None, 4)}).div(gain));
    return boxes;
}

// Classes 
std::vector<std::string> classes {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
                                  "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
                                  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                                  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife",
                                  "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
                                  "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
                                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

int capture_width  = 640;		// capture_width = 1280 ;
int capture_height = 480;		// capture_height = 720 ;
int display_width  = 640;		// display_width = 1280 ;
int display_height = 480;		// display_height = 720 ;
int framerate      = 20;	    // framerate = 30 ;
int flip_method    = 0;

cv::Mat image;
cv::Mat input_image;
cv::VideoCapture video_stream;

const char *urlGPS = "https://bear-alert-828c1-default-rtdb.firebaseio.com/Bear_Alert/Fixed_point_33/GPS.json";
const char *urlNo  = "https://bear-alert-828c1-default-rtdb.firebaseio.com/Bear_Alert/Fixed_point_33/bearsNo.json";

int skipFrame            = 0;
int number_of_bears      = 0;
int number_of_bears_keep = 0;

pthread_t tid_WriteDB;
sem_t semWrDB;
int runningDB;

int main() 
{
	runningDB = 1;
	
    // Device
    torch::Device device(torch::cuda::is_available() ? torch::kCUDA :torch::kCPU);

    torch::jit::script::Module yolo_model;

    try 
    {
		// Create the exit point
		if ( signal (SIGINT, sig_handler_INT) == SIG_ERR )
			fprintf (stderr, "Can't catch SIGINT. Error!");
		
		// Create thread to write in the Firebase DB
		sem_init (&semWrDB, 0, 0); 

		if (pthread_create (&tid_WriteDB, NULL, &writeDB, NULL) != 0)
			{
			fprintf(stderr, "Failed to create the thread required to write in Firebase DB.\n");	
			return 3;
			}
		else
			printf ("The thread which will write in Firebase DB was created!.\n");	
				
		
        // Load the model (e.g. yolov11n.torchscript)
        std::string model_path = "../yolo11n.torchscript";
        yolo_model = torch::jit::load(model_path, device);
        yolo_model.eval();
        yolo_model.to(device, torch::kFloat32);
        
		std::string pipeline;
		pipeline = gstreamer_pipeline(
						capture_width,
						capture_height,
						display_width,
						display_height,
						framerate,
						flip_method
						);
									
		// Open camera 
        video_stream.open(pipeline, cv::CAP_GSTREAMER);

        if (!video_stream.isOpened()) 
			{
			std::cout << "Failed to open camera" << std::endl;	
			
			return 1;
			}
		else
			std::cout << "Camera was succesfully open" << std::endl;	
    } 
    catch (const c10::Error& e) 
		{
        std::cout << e.msg() << std::endl;
		}

    while (1)
    {
		try 
		{
			// Load image and preprocess
			//cv::Mat image = cv::imread("../bus.jpg");
			if (!video_stream.read(image)) 
				{
				std::cout << "Frame capture read error" << std::endl;	
				
				return 2;
				}        
			
			letterbox(image, input_image, {640, 640});
			cv::cvtColor(input_image, input_image, cv::COLOR_BGR2RGB);

			torch::Tensor image_tensor = torch::from_blob(input_image.data, {input_image.rows, input_image.cols, 3}, torch::kByte).to(device);
			image_tensor = image_tensor.toType(torch::kFloat32).div(255);
			image_tensor = image_tensor.permute({2, 0, 1});
			image_tensor = image_tensor.unsqueeze(0);
			std::vector<torch::jit::IValue> inputs {image_tensor};

			// Inference
			torch::Tensor output = yolo_model.forward(inputs).toTensor().cpu();

			// NMS
			auto keep  = non_max_suppression(output)[0];
			auto boxes = keep.index({Slice(), Slice(None, 4)});
			keep.index_put_({Slice(), Slice(None, 4)}, scale_boxes({input_image.rows, input_image.cols}, boxes, {image.rows, image.cols}));

			number_of_bears = 0;
			
			// Show the results
			for (int i = 0; i < keep.size(0); i++) 
			{
				int x1     = keep[i][0].item().toFloat();
				int y1     = keep[i][1].item().toFloat();
				int x2     = keep[i][2].item().toFloat();
				int y2     = keep[i][3].item().toFloat();
				float conf = keep[i][4].item().toFloat();
				int cls    = keep[i][5].item().toInt();
		
				// confidence threshold
				if (conf < 0.5) continue; 
				
				if (cls  != 0 ) continue; 

				std::cout << "Rect: [" << x1 << "," << y1 << "," << x2 << "," << y2 << "]  Conf: " << conf << "  Class: " << classes[cls] << std::endl;
				
				if (cls == 0)
					{number_of_bears ++;}
				
				cv::Scalar color = get_color(cls);
				
				cv::rectangle(image,
							  cv::Point((int)x1, (int)y1),
							  cv::Point((int)x2, (int)y2),
							  color, 4);
					
				// optionally label: cls, etc.
				std::ostringstream label_ss;
				label_ss << std::setprecision(2);
				
				label_ss << classes[cls] << " " << conf;
				
				cv::putText(image, label_ss.str(),
							cv::Point((int)x1, (int)y1 - 5),
							cv::FONT_HERSHEY_SIMPLEX, 1.0,
							color, 2);
			}
			
			// Show or save the result
			cv::imshow("Detections", image);
			char key = (char) cv::waitKey(5); 			//wait 5 ms for a key press
			
			if (key == 27 || key == 'q')				// ESK or 'q'
				break;
				
		//========= Firebase START =========================================================		
			if 	( (number_of_bears > 0) && (skipFrame == 0) )
				{
				skipFrame = 1;
				number_of_bears_keep = number_of_bears;
				
				sem_post(&semWrDB);
				}	
				
			if (skipFrame > 0)
				{
				skipFrame++;
				printf ("skipFrame = %d\n", skipFrame);
				
				if (skipFrame > 300)	
					skipFrame = 0;
				}	
		//========= Firebase START =========================================================		
		
			sleep(0.030);	
			
		} 
		catch (const c10::Error& e) 
			{
			std::cout << e.msg() << std::endl;
			}
	}
	
	video_stream.release();
	cv::destroyAllWindows();
	
	runningDB = 0;
	sem_post(&semWrDB);
	pthread_join(tid_WriteDB, NULL);
	
	sem_destroy(&semWrDB);	

    return 0;
}

void sig_handler_INT (int num)
{
	printf ("\nEnter in close procedure.\n\n");
	
	video_stream.release();
	cv::destroyAllWindows();
	
	runningDB = 0;
	sem_post(&semWrDB);
	pthread_join(tid_WriteDB, NULL);
	
	sem_destroy(&semWrDB);

	printf ("\n\nPROGRAM WAS ENDED!\n\n");

    exit (0);
}


void* writeDB(void *arg)
{
	while (runningDB)
		{
		sem_wait(&semWrDB);	
		
		if (runningDB)
			{
			printf ("\n\n\nENTERD in write DB!\n\n\n");	
			// Write GPS coordinate into Bear_Alert/MY_STRING
			if (!writeStringToFirebase(urlGPS, "47.177245, 27.567160"))
				{
				if (!writeStringToFirebase(urlGPS, "47.177245, 27.567160"))
					fprintf(stderr, "Failed to write the GPS values.");
				else
					printf( "\nI writed [2] to Firebase the GPS values.\n");
				}
			else
				printf( "\nI writed [1] to Firebase the GPS values.\n");
				
			if ( !writeIntToFirebase(urlNo, number_of_bears_keep) ) 
				{
				if ( !writeIntToFirebase(urlNo, number_of_bears_keep) ) 
					fprintf(stderr, "Failed to write the number into Firebase DB.\n");
				else
					printf( "\nI writed [2] to Firebase the number of detections.\n");
				}
			else
				printf( "\nI writed [1] to Firebase the number of detections.\n");
			}	
		}
		
	pthread_exit(NULL);	
}

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) 
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

// Define some colors for classes
cv::Scalar get_color(int class_id) 
{
    static cv::Scalar colors[] = 
    {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(0, 0, 255),   // Red
        cv::Scalar(255, 255, 0), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255)  // Yellow
    };
    
    return colors[class_id % 6]; // cycle if >6 classes
}

// Write a string into Firebase at the given path
int writeStringToFirebase(const char *url, const char *value) 
{
    CURL *curl;
    CURLcode res;
    int success = 0;

    // Wrap the value in quotes (JSON string)
    char jsonBody[512];
    snprintf(jsonBody, sizeof(jsonBody), "\"%s\"", value);

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (curl) 
    {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonBody);

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) 
			{
            printf("Successfully wrote: %s\n", value);
            success = 1;
			} 
        else 
			{
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
			}

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();

    return success;
}


// Write an integer value into Firebase at the given path
int writeIntToFirebase(const char *url, int value) 
{
    CURL *curl;
    CURLcode res;
    int success = 0;

    // Prepare JSON body: integer (no quotes)
    char jsonBody[64];
    snprintf(jsonBody, sizeof(jsonBody), "%d", value);

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonBody);

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            success = 1;
            printf("Successfully wrote number %d to Firebase.\n", value);
        } else {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                    curl_easy_strerror(res));
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();

    return success;
}

