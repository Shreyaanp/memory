#include "face_mesh_wrapper.h"
#include <Python.h>
#include <numpy/arrayobject.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>

namespace mdai {

// Static initialization for Python
static std::once_flag python_init_flag;
static PyObject* mediapipe_module = nullptr;
static PyObject* solutions_module = nullptr;
static PyObject* face_mesh_class = nullptr;

// Helper function for NumPy initialization (needed because import_array is a macro)
static int init_numpy() {
    import_array();
    return 0;
}

/**
 * Initialize Python interpreter and MediaPipe modules
 */
static bool InitializePython() {
    static bool initialized = false;
    if (initialized) return true;
    
    std::call_once(python_init_flag, []() {
        std::cout << "[MediaPipe] Initializing Python interpreter..." << std::endl;
        
        Py_Initialize();
        
        if (!Py_IsInitialized()) {
            std::cerr << "[MediaPipe] Failed to initialize Python" << std::endl;
            return;
        }
        
        // Initialize NumPy C API
        if (init_numpy() < 0) {
            std::cerr << "[MediaPipe] Failed to initialize NumPy" << std::endl;
            return;
        }
        
        // Import MediaPipe
        mediapipe_module = PyImport_ImportModule("mediapipe");
        if (!mediapipe_module) {
            PyErr_Print();
            std::cerr << "[MediaPipe] Failed to import mediapipe" << std::endl;
            return;
        }
        
        // Get solutions module
        solutions_module = PyObject_GetAttrString(mediapipe_module, "solutions");
        if (!solutions_module) {
            PyErr_Print();
            std::cerr << "[MediaPipe] Failed to get solutions module" << std::endl;
            return;
        }
        
        // Get face_mesh module
        PyObject* face_mesh_module = PyObject_GetAttrString(solutions_module, "face_mesh");
        if (!face_mesh_module) {
            PyErr_Print();
            std::cerr << "[MediaPipe] Failed to get face_mesh module" << std::endl;
            return;
        }
        
        // Get FaceMesh class
        face_mesh_class = PyObject_GetAttrString(face_mesh_module, "FaceMesh");
        if (!face_mesh_class) {
            PyErr_Print();
            std::cerr << "[MediaPipe] Failed to get FaceMesh class" << std::endl;
            Py_DECREF(face_mesh_module);
            return;
        }
        
        Py_DECREF(face_mesh_module);
        initialized = true;
        std::cout << "[MediaPipe] Python initialization complete" << std::endl;
    });
    
    return initialized;
}

/**
 * Convert OpenCV Mat to NumPy array
 */
static PyObject* Mat2NumPy(const cv::Mat& image) {
    npy_intp dimensions[3] = {image.rows, image.cols, image.channels()};
    
    // Create NumPy array (this creates a copy)
    PyObject* array = PyArray_SimpleNew(3, dimensions, NPY_UINT8);
    if (!array) {
        PyErr_Print();
        return nullptr;
    }
    
    // Copy data
    uint8_t* dst = (uint8_t*)PyArray_DATA((PyArrayObject*)array);
    memcpy(dst, image.data, image.total() * image.elemSize());
    
    return array;
}

// ============================================================================
// FaceMeshDetector::Impl - PIMPL Implementation
// ============================================================================

class FaceMeshDetector::Impl {
public:
    Impl(const FaceMeshConfig& config) 
        : config_(config)
        , face_mesh_instance_(nullptr)
        , initialized_(false)
        , last_error_("Not initialized")
    {
        Initialize();
    }
    
    ~Impl() {
        if (face_mesh_instance_) {
            Py_DECREF(face_mesh_instance_);
        }
    }
    
    bool Initialize() {
        if (initialized_) return true;
        
        if (!InitializePython()) {
            last_error_ = "Failed to initialize Python";
            return false;
        }
        
        if (!face_mesh_class) {
            last_error_ = "FaceMesh class not available";
            return false;
        }
        
        // Create FaceMesh instance with parameters
        PyObject* kwargs = PyDict_New();
        PyDict_SetItemString(kwargs, "max_num_faces", PyLong_FromLong(config_.num_faces));
        PyDict_SetItemString(kwargs, "refine_landmarks", PyBool_FromLong(1));
        PyDict_SetItemString(kwargs, "min_detection_confidence", 
                            PyFloat_FromDouble(config_.min_detection_confidence));
        PyDict_SetItemString(kwargs, "min_tracking_confidence", 
                            PyFloat_FromDouble(config_.min_tracking_confidence));
        
        // Static image mode = False for video processing
        PyDict_SetItemString(kwargs, "static_image_mode", PyBool_FromLong(0));
        
        face_mesh_instance_ = PyObject_Call(face_mesh_class, PyTuple_New(0), kwargs);
        Py_DECREF(kwargs);
        
        if (!face_mesh_instance_) {
            PyErr_Print();
            last_error_ = "Failed to create FaceMesh instance";
            return false;
        }
        
        initialized_ = true;
        last_error_ = "";
        std::cout << "[MediaPipe] FaceMesh detector initialized with GPU support" << std::endl;
        return true;
    }
    
    bool Detect(const cv::Mat& bgr_image, FaceMeshResult& result) {
        if (!initialized_) {
            last_error_ = "Detector not initialized";
            return false;
        }
        
        if (bgr_image.empty()) {
            last_error_ = "Empty input image";
            return false;
        }
        
        // Convert BGR to RGB
        cv::Mat rgb_image;
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
        
        // Convert to NumPy array
        PyObject* numpy_image = Mat2NumPy(rgb_image);
        if (!numpy_image) {
            last_error_ = "Failed to convert image to NumPy";
            return false;
        }
        
        // Call process() method
        PyObject* process_method = PyObject_GetAttrString(face_mesh_instance_, "process");
        if (!process_method) {
            PyErr_Print();
            Py_DECREF(numpy_image);
            last_error_ = "Failed to get process method";
            return false;
        }
        
        PyObject* args = PyTuple_Pack(1, numpy_image);
        PyObject* process_result = PyObject_CallObject(process_method, args);
        
        Py_DECREF(args);
        Py_DECREF(numpy_image);
        Py_DECREF(process_method);
        
        if (!process_result) {
            PyErr_Print();
            last_error_ = "Process method failed";
            return false;
        }
        
        // Parse results
        bool success = ParseResults(process_result, rgb_image.cols, rgb_image.rows, result);
        Py_DECREF(process_result);
        
        return success;
    }
    
    bool IsInitialized() const {
        return initialized_;
    }
    
    std::string GetLastError() const {
        return last_error_;
    }
    
private:
    bool ParseResults(PyObject* results, int width, int height, FaceMeshResult& result) {
        // Clear previous results
        result.landmarks.clear();
        result.visibility.clear();
        result.presence.clear();
        result.confidence = 0.0f;
        result.image_width = width;
        result.image_height = height;
        
        // Check if multi_face_landmarks exists
        PyObject* multi_face_landmarks = PyObject_GetAttrString(results, "multi_face_landmarks");
        if (!multi_face_landmarks || multi_face_landmarks == Py_None) {
            Py_XDECREF(multi_face_landmarks);
            last_error_ = "No face detected";
            return true; // Not an error, just no face
        }
        
        // Get list length
        Py_ssize_t num_faces = PyList_Size(multi_face_landmarks);
        if (num_faces == 0) {
            Py_DECREF(multi_face_landmarks);
            last_error_ = "No face detected";
            return true;
        }
        
        // Get first face landmarks
        PyObject* face_landmarks = PyList_GetItem(multi_face_landmarks, 0); // Borrowed reference
        if (!face_landmarks) {
            Py_DECREF(multi_face_landmarks);
            last_error_ = "Failed to get face landmarks";
            return false;
        }
        
        // Get landmark list
        PyObject* landmark_list = PyObject_GetAttrString(face_landmarks, "landmark");
        if (!landmark_list) {
            PyErr_Print();
            Py_DECREF(multi_face_landmarks);
            last_error_ = "Failed to get landmark list";
            return false;
        }
        
        Py_ssize_t num_landmarks = PyList_Size(landmark_list);
        
        // Reserve space
        result.landmarks.reserve(num_landmarks);
        result.visibility.reserve(num_landmarks);
        result.presence.reserve(num_landmarks);
        
        // Parse each landmark
        float min_x = 1.0f, min_y = 1.0f, max_x = 0.0f, max_y = 0.0f;
        
        for (Py_ssize_t i = 0; i < num_landmarks; i++) {
            PyObject* landmark = PyList_GetItem(landmark_list, i); // Borrowed reference
            
            // Get x, y, z (normalized coordinates 0-1)
            PyObject* x_obj = PyObject_GetAttrString(landmark, "x");
            PyObject* y_obj = PyObject_GetAttrString(landmark, "y");
            PyObject* z_obj = PyObject_GetAttrString(landmark, "z");
            PyObject* vis_obj = PyObject_GetAttrString(landmark, "visibility");
            PyObject* pres_obj = PyObject_GetAttrString(landmark, "presence");
            
            float x_norm = PyFloat_AsDouble(x_obj);
            float y_norm = PyFloat_AsDouble(y_obj);
            float z_norm = PyFloat_AsDouble(z_obj);
            
            // Convert to pixel coordinates
            float x_pix = x_norm * width;
            float y_pix = y_norm * height;
            float z_pix = z_norm * width; // Z is also normalized
            
            result.landmarks.push_back(cv::Point3f(x_pix, y_pix, z_pix));
            
            // Visibility and presence (may be None)
            float visibility = (vis_obj && vis_obj != Py_None) ? PyFloat_AsDouble(vis_obj) : 1.0f;
            float presence = (pres_obj && pres_obj != Py_None) ? PyFloat_AsDouble(pres_obj) : 1.0f;
            
            result.visibility.push_back(visibility);
            result.presence.push_back(presence);
            
            // Update bounding box
            if (x_norm < min_x) min_x = x_norm;
            if (x_norm > max_x) max_x = x_norm;
            if (y_norm < min_y) min_y = y_norm;
            if (y_norm > max_y) max_y = y_norm;
            
            Py_XDECREF(x_obj);
            Py_XDECREF(y_obj);
            Py_XDECREF(z_obj);
            Py_XDECREF(vis_obj);
            Py_XDECREF(pres_obj);
        }
        
        // Calculate bounding box
        result.bbox = cv::Rect(
            static_cast<int>(min_x * width),
            static_cast<int>(min_y * height),
            static_cast<int>((max_x - min_x) * width),
            static_cast<int>((max_y - min_y) * height)
        );
        
        // Estimate confidence from presence scores
        float avg_presence = 0.0f;
        for (float p : result.presence) {
            avg_presence += p;
        }
        result.confidence = avg_presence / result.presence.size();
        
        Py_DECREF(landmark_list);
        Py_DECREF(multi_face_landmarks);
        
        last_error_ = "";
        return true;
    }
    
    FaceMeshConfig config_;
    PyObject* face_mesh_instance_;
    bool initialized_;
    std::string last_error_;
};

// ============================================================================
// FaceMeshResult Implementation
// ============================================================================

bool FaceMeshResult::IsOccluded(float visibility_threshold, float occlusion_ratio) const {
    if (visibility.empty()) return false;
    
    int occluded_count = 0;
    for (float vis : visibility) {
        if (vis < visibility_threshold) {
            occluded_count++;
        }
    }
    
    float ratio = static_cast<float>(occluded_count) / visibility.size();
    return ratio > occlusion_ratio;
}

std::vector<cv::Point3f> FaceMeshResult::GetRegionLandmarks(const std::vector<int>& indices) const {
    std::vector<cv::Point3f> region;
    region.reserve(indices.size());
    
    for (int idx : indices) {
        if (idx >= 0 && idx < static_cast<int>(landmarks.size())) {
            region.push_back(landmarks[idx]);
        }
    }
    
    return region;
}

bool FaceMeshResult::IsLandmarkValid(int index, float min_visibility) const {
    if (index < 0 || index >= static_cast<int>(landmarks.size())) {
        return false;
    }
    
    if (index >= static_cast<int>(visibility.size())) {
        return true; // No visibility info, assume valid
    }
    
    return visibility[index] >= min_visibility;
}

// Landmark indices (MediaPipe standard 468-point topology)
const std::vector<int> FaceMeshResult::LandmarkIndices::FACE_OVAL = {
    10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
    397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
    172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_EYE = {
    33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_EYE = {
    263, 249, 390, 373, 374, 380, 381, 382, 362, 398, 384, 385, 386, 387, 388, 466
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LIPS_OUTER = {
    61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291, 308
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LIPS_INNER = {
    78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308, 324
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_EYEBROW = {
    46, 53, 52, 65, 55, 70, 63, 105
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_EYEBROW = {
    276, 283, 282, 295, 285, 300, 293, 334
};

const std::vector<int> FaceMeshResult::LandmarkIndices::NOSE = {
    1, 2, 98, 327, 4, 5, 6, 168, 197
};

const std::vector<int> FaceMeshResult::LandmarkIndices::FOREHEAD = {
    10, 338, 297, 332, 284, 251, 389, 356, 454, 323
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_CHEEK = {
    50, 117, 118, 119, 120, 121, 122, 123, 126, 203
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_CHEEK = {
    280, 346, 347, 348, 349, 350, 351, 352, 355, 423
};

// ============================================================================
// FaceMeshDetector Implementation
// ============================================================================

FaceMeshDetector::FaceMeshDetector(const FaceMeshConfig& config) 
    : impl_(std::make_unique<Impl>(config))
{
}

FaceMeshDetector::~FaceMeshDetector() = default;

bool FaceMeshDetector::Detect(const cv::Mat& rgb_image, FaceMeshResult& result) {
    return impl_->Detect(rgb_image, result);
}

bool FaceMeshDetector::IsInitialized() const {
    return impl_->IsInitialized();
}

std::string FaceMeshDetector::GetLastError() const {
    return impl_->GetLastError();
}

}  // namespace mdai

