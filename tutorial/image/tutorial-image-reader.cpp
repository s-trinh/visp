#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/core/vpImageTools.h>

namespace
{
void merge(const vpImage<vpRGBa> &I_demo, const vpImage<vpRGBa> &I_blinding_resize, vpImage<vpRGBa> &I_out,
  int bb_top, int bb_left, int bb_w, int bb_h, float coeff_a_, float coeff_b_)
{
  int smooth_h = 20, smooth_w = 20;
  for (int i = bb_top; i < bb_h; i++) {
    for (int j = bb_left; j < bb_w; j++) {
      vpRGBa a = I_demo[i][j];
      vpRGBa b = I_blinding_resize[i][j];
      float coeff_a = 0;
      float coeff_b = coeff_b_;
      if (i >= bb_h-smooth_h) {
        float offset = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
        float smooth_coeff = std::tanh(3*offset); // std::tanh(offset);
        coeff_a = smooth_coeff*coeff_a_; // (1.0f - smooth_coeff)*coeff_a_;
        // std::cout << "smooth_coeff=" << smooth_coeff << " ; coeff_a=" << coeff_a << " ; coeff_b=" << coeff_b << std::endl;
      }
      else {
        coeff_a = coeff_a_; // (1.0f - std::tanh(1))*coeff_a_;
      }
      vpRGBa c = coeff_a*a + coeff_b*b;
      I_out[i][j] = c;
    }
  }
}

void computeSmoothingTable(vpImage<unsigned char> &I_smooth, int smooth_h, int smooth_w, int bb_top, int bb_left, int bb_w, int bb_h)
{
  for (int i = bb_top; i < bb_h; i++) {
    float coeff = 0;
    if (i >= bb_h-smooth_h) {
      float offset = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
      float smooth_coeff = std::tanh(3*offset);
      coeff = smooth_coeff;
    }

    for (int j = bb_left; j < bb_w; j++) {
      float coeff_b = 0;
      if (i < bb_h-smooth_h && j >= bb_w-smooth_w) {
        float offset = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
        float smooth_coeff = std::tanh(3*offset);
        coeff = smooth_coeff;
      }
      else if (i >= bb_h-smooth_h && j >= bb_w-smooth_w) {
        float dist_h = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
        float dist_w = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
        float offset = std::sqrt(dist_h*dist_h + dist_w*dist_w) / std::sqrt(2);
        float smooth_coeff = std::tanh(3*offset);
        coeff = smooth_coeff;
      }
      I_smooth[i][j] = coeff*255;
    }
  }
}

void merge(const vpImage<vpRGBa> &I_demo, const vpImage<vpRGBa> &I_blinding_resize, vpImage<vpRGBa> &I_out,
  int bb_top, int bb_left, int bb_w, int bb_h, float coeff_a_, float coeff_b_, const vpImage<unsigned char> &I_smooth)
{
  // int smooth_h = 20, smooth_w = 20;
  for (int i = bb_top; i < bb_h; i++) {
    for (int j = bb_left; j < bb_w; j++) {
      vpRGBa a = I_demo[i][j];
      vpRGBa b = I_blinding_resize[i][j];
      float coeff_a = coeff_a_ * (I_smooth[i][j] / 255.0f);
      float coeff_b = coeff_b_;
      vpRGBa c = coeff_a*a + coeff_b*b;
      I_out[i][j] = c;
    }
  }
}
}

int main()
{
  // vpImage<vpRGBa> I;
  // vpVideoReader g;
  // // g.setFileName(""); // TODO:
  // g.setFileName(""); // TODO:
  // int start_index = (2*60 + 40) * 30;
  // g.setFirstFrameIndex(start_index);
  // g.open(I);

  // int index = 0;
  // while (!g.end()) {
  //   g.acquire(I);

  //   char buffer[FILENAME_MAX];
  //   // const std::string output_dir = "rush_3/demo/"; // TODO:
  //   const std::string output_dir = "rush_3/lights/"; // TODO:
  //   snprintf(buffer, FILENAME_MAX, std::string(output_dir + "image_%04d.png").c_str(), index);
  //   const std::string output_filename = buffer;

  //   // std::cout << "I: " << I.getWidth() << "x" << I.getHeight() << std::endl;
  //   // std::cout << "output_filename=" << output_filename << std::endl;
  //   vpImageIo::write(I, output_filename);

  //   index++;

  //   const int end_index = 30*30;
  //   if (index > end_index) {
  //     break;
  //   }
  // }







#if defined DEBUG_VISU_SMOOTHING
  int bb_top = 0, bb_left = 0, bb_w = 600, bb_h = 200;
  int smooth_h = 100, smooth_w = 100;
  vpImage<unsigned char> I_smooth(720, 1280, 255);
  // for (int i = bb_top; i < bb_h; i++) {
  //   float coeff_a = 0;
  //   if (i >= bb_h-smooth_h) {
  //     float offset = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
  //     float smooth_coeff = std::tanh(3*offset);
  //     coeff_a = smooth_coeff;
  //   }

  //   for (int j = bb_left; j < bb_w; j++) {
  //     float coeff_b = 0;
  //     if (j >= bb_w-smooth_w) {
  //       float offset = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
  //       float smooth_coeff = std::tanh(3*offset);
  //       coeff_b = smooth_coeff;
  //     }
  //     I_smooth[i][j] = (coeff_a+coeff_b)*255;
  //   }
  // }
  // for (int i = bb_top; i < bb_h; i++) {
  //   for (int j = bb_left; j < bb_w; j++) {
  //     float dist_h = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
  //     float dist_w = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
  //     float dist = std::sqrt(dist_h*dist_h + dist_w*dist_w) / std::sqrt(2);
  //     // if (j >= bb_w-smooth_w) {
  //     //   float offset = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
  //     //   float smooth_coeff = std::tanh(3*offset);
  //     //   coeff_b = smooth_coeff;
  //     // }
  //     // I_smooth[i][j] = std::tanh(3*dist)*255;
  //     I_smooth[i][j] = dist*255;
  //   }
  // }
  for (int i = bb_top; i < bb_h; i++) {
    float coeff = 0;
    if (i >= bb_h-smooth_h) {
      float offset = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
      float smooth_coeff = std::tanh(3*offset);
      coeff = smooth_coeff;
    }

    for (int j = bb_left; j < bb_w; j++) {
      float coeff_b = 0;
      if (i < bb_h-smooth_h && j >= bb_w-smooth_w) {
        float offset = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
        float smooth_coeff = std::tanh(3*offset);
        coeff = smooth_coeff;
      }
      else if (i >= bb_h-smooth_h && j >= bb_w-smooth_w) {
        float dist_h = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
        float dist_w = ((j+1) - (bb_w-smooth_w)) / float(smooth_w);
        float offset = std::sqrt(dist_h*dist_h + dist_w*dist_w) / std::sqrt(2);
        float smooth_coeff = std::tanh(3*offset);
        coeff = smooth_coeff;
      }
      I_smooth[i][j] = coeff*255;
    }
  }
  vpImageIo::write(I_smooth, "debug_smoothing_func.png");
#endif








  const int final_w = 1920, final_h = 1080;
  vpImage<unsigned char> I_smooth(final_h, final_w, 255);

  int start_index_demo = 1134;
  int end_index_demo = 205;

  int start_index_blinding = 75;
  int end_index_blinding = 520;

  float resize_factor = 1920.0f / 1280;

  vpImage<vpRGBa> I_demo;
  vpImage<vpRGBa> I_blinding;
  vpImage<vpRGBa> I_blinding_resize(final_h, final_w);
  vpImage<vpRGBa> I_out;
  char buffer[FILENAME_MAX];

  int index_out = 0;
  int index_blinding = start_index_blinding;
  for (int index_demo = start_index_demo /* - (end_index_blinding - start_index_blinding)*/;
      index_demo >= end_index_demo; index_demo--, index_blinding++, index_out++) {

    const std::string demo_dir = "rush_3/demo/"; // TODO:
    snprintf(buffer, FILENAME_MAX, std::string(demo_dir + "image_%04d.png").c_str(), index_demo);
    const std::string demo_filename = buffer;
    // std::cout << "demo_filename=" << demo_filename << std::endl;
    vpImageIo::read(I_demo, demo_filename);
    I_out = I_demo;

    const std::string blinding_dir = "rush_3/lights/"; // TODO:
    snprintf(buffer, FILENAME_MAX, std::string(blinding_dir + "image_%04d.png").c_str(), index_blinding);
    const std::string blinding_filename = buffer;
    // std::cout << "blinding_filename=" << blinding_filename << std::endl;
    vpImageIo::read(I_blinding, blinding_filename);
    // vpImageTools::resize(I_blinding, I_blinding_resize, vpImageTools::INTERPOLATION_LINEAR);
    I_blinding_resize = I_blinding;

    // BB1
    const int bb1_top = 0;
    const int bb1_left = 0;
    const int bb1_width = 1280;
    const int bb1_height = 170;

    // BB2
    const int bb2_top = 0;
    const int bb2_left = 0;
    const int bb2_width = 990;
    const int bb2_height = 470;

    // if (index_out <= end_index_blinding-start_index_blinding) {
    if (index_demo <= start_index_demo - (end_index_blinding - start_index_blinding)) {
      const float idx_ratio = (index_out - (start_index_demo - index_demo)) / (float)(end_index_blinding - start_index_blinding);
      const int bb_width = (bb2_width - bb1_width) * idx_ratio + bb1_width;
      const int bb_height = (bb2_height - bb1_height) * idx_ratio + bb1_height;
      // std::cout << "idx_ratio=" << idx_ratio << " ; bb_size=" << bb_width << "x" << bb_height << std::endl;

      // TODO:
      resize_factor = 1.0f;

      const float coeff_a = 0.6, coeff_b = 0.4;
      const int smooth_h = bb_height, smooth_w = bb_width;
      computeSmoothingTable(I_smooth, smooth_h, smooth_w, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor);
      // merge(I_demo, I_blinding_resize, I_out, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor, 0.6, 0.4);
      merge(I_demo, I_blinding_resize, I_out, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor, coeff_a, coeff_b, I_smooth);
    }

    const std::string output_dir = "rush_3/out/";
    snprintf(buffer, FILENAME_MAX, std::string(output_dir + "image_%04d.png").c_str(), index_out);
    const std::string output_filename = buffer;
    // std::cout << "output_filename=" << output_filename << std::endl;
    vpImageIo::write(I_out, output_filename);
  }

  return 0;
}
