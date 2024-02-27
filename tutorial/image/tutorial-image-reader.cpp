#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/core/vpImageTools.h>

namespace
{
// void merge(const vpImage<vpRGBa> &I_demo, const vpImage<vpRGBa> &I_blinding_resize, vpImage<vpRGBa> &I_out,
//   int bb_top, int bb_left, int bb_w, int bb_h, float coeff_a_, float coeff_b_)
// {
//   int smooth_h = 20, smooth_w = 20;
//   for (int i = bb_top; i < bb_h; i++) {
//     for (int j = bb_left; j < bb_w; j++) {
//       vpRGBa a = I_demo[i][j];
//       vpRGBa b = I_blinding_resize[i][j];
//       float coeff_a = 0;
//       float coeff_b = coeff_b_;
//       if (i >= bb_h-smooth_h) {
//         float offset = ((i+1) - (bb_h-smooth_h)) / float(smooth_h);
//         float smooth_coeff = std::tanh(3*offset); // std::tanh(offset);
//         coeff_a = smooth_coeff*coeff_a_; // (1.0f - smooth_coeff)*coeff_a_;
//         // std::cout << "smooth_coeff=" << smooth_coeff << " ; coeff_a=" << coeff_a << " ; coeff_b=" << coeff_b << std::endl;
//       }
//       else {
//         coeff_a = coeff_a_; // (1.0f - std::tanh(1))*coeff_a_;
//       }
//       vpRGBa c = coeff_a*a + coeff_b*b;
//       I_out[i][j] = c;
//     }
//   }
// }

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

void computeSmoothingTable2(vpImage<unsigned char> &I_smooth, int smooth_h, int smooth_w, int bb_top, int bb_left, int bb_w, int bb_h,
  float coeff_a_)
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

      float coeff_a = coeff * (1 - coeff_a_) + coeff_a_;

      I_smooth[i][j] = coeff_a*255;
    }
  }
}

void smooth(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &I_out, const vpImage<unsigned char> &I_smooth,
  int bb_top, int bb_left, int bb_w, int bb_h)
{
  for (int i = bb_top; i < bb_h; i++) {
    for (int j = bb_left; j < bb_w; j++) {
      vpRGBa a = I[i][j];
      float coeff_smooth = I_smooth[i][j] / 255.0f;
      I_out[i][j] = coeff_smooth * a;
    }
  }
}

void merge(const vpImage<vpRGBa> &I_demo, const vpImage<vpRGBa> &I_blinding_resize, vpImage<vpRGBa> &I_out,
  int bb_top, int bb_left, int bb_w, int bb_h, float coeff_a_, float coeff_b_, const vpImage<unsigned char> &I_smooth)
{
  for (int i = bb_top; i < bb_h; i++) {
    for (int j = bb_left; j < bb_w; j++) {
      vpRGBa a = I_demo[i][j];
      vpRGBa b = I_blinding_resize[i][j];
      float coeff_smooth = I_smooth[i][j] / 255.0f;

      float coeff_a = coeff_smooth * (1 - coeff_a_) + coeff_a_;
      // float coeff_b = (1.0f - coeff_smooth) * coeff_b_;
      float coeff_b = 1.0f - coeff_a;
      vpRGBa c = coeff_a*a + coeff_b*b;
      I_out[i][j] = c;
    }
  }
}

void mergeSimple(const vpImage<vpRGBa> &I_demo, const vpImage<vpRGBa> &I_blinding_resize, vpImage<vpRGBa> &I_out,
  const vpImage<unsigned char> &I_coeff)
{
  for (unsigned int i = 0; i < I_demo.getRows(); i++) {
    for (unsigned int j = 0; j < I_demo.getCols(); j++) {
      float coeff_smooth = I_coeff[i][j] / 255.0f;
      vpRGBa a = I_demo[i][j];
      vpRGBa b = I_blinding_resize[i][j];

      I_out[i][j] = coeff_smooth*a + (1 - coeff_smooth)*b;
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







  // // Extract all
  // vpImage<vpRGBa> I;
  // vpVideoReader g;
  // g.setFileName("rush_3/demarlus_rush_2.mp4");
  // g.open(I);

  // int index = 0;
  // while (!g.end()) {
  //   g.acquire(I);

  //   char buffer[FILENAME_MAX];
  //   const std::string output_dir = "rush_3/in/"; // TODO:
  //   snprintf(buffer, FILENAME_MAX, std::string(output_dir + "image_%04d.png").c_str(), index);
  //   const std::string output_filename = buffer;

  //   vpImageIo::write(I, output_filename);

  //   index++;
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








#define DEBUG 0
#define MERGE 1
#if MERGE
  const int final_w = 1920, final_h = 1080;
  vpImage<unsigned char> I_smooth(final_h, final_w, 255);
  vpImage<unsigned char> I_smooth2(final_h, final_w, 255);

  int start_index_demo = 1134;
  int end_index_demo = 205;

  int start_index_blinding = 75;
  int end_index_blinding = 520;

  float resize_factor = 1920.0f / 1280;

  vpImage<vpRGBa> I_demo;
  vpImage<vpRGBa> I_blinding;
  vpImage<vpRGBa> I_blinding_resize(final_h, final_w);
  vpImage<vpRGBa> I_blinding_resize_start(final_h, final_w);
  vpImage<vpRGBa> I_out;
  char buffer[FILENAME_MAX];

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

  // Merge coeffs
  const float coeff_a = 0.4, coeff_b = 0.6;
  // Smoothing window size
  const int smooth_h = 100, smooth_w = 100;

  // Try to have smoothing transition temporally
  const int nb_frames_temporal_merge = 60;
  vpImage<unsigned char> I_smooth_transition(final_h, final_w, 255);
  computeSmoothingTable2(I_smooth_transition, smooth_h, smooth_w, bb1_top, bb1_left, bb1_width*resize_factor, bb1_height*resize_factor, coeff_a);

  // Read the first image of the second video
  {
    const std::string blinding_dir = "rush_3/lights/"; // TODO:
    snprintf(buffer, FILENAME_MAX, std::string(blinding_dir + "image_%04d.png").c_str(), start_index_blinding);
    const std::string blinding_filename = buffer;
    vpImageIo::read(I_blinding, blinding_filename);
    vpImageTools::resize(I_blinding, I_blinding_resize_start, vpImageTools::INTERPOLATION_LINEAR);
  }

  // // TODO:
  // {
  //   const std::string output_dir = "rush_3/debug/";
  //   vpImageIo::write(I_smooth_transition, output_dir + "I_smooth_transition.png");
  // }
  // {
  //   const std::string output_dir = "rush_3/debug/";
  //   vpImageIo::write(I_blinding_resize_start, output_dir + "I_blinding_resize_start.png");
  // }

  // TODO:
  int index_out = end_index_demo;
  int index_out_start = end_index_demo + (end_index_blinding - start_index_blinding);
  int index_blinding = start_index_blinding;
  std::cout << "index_out=" << index_out << " ; index_out_start=" << index_out_start << " ; index_blinding="
    << index_blinding << " ; index_temporal=" << (index_out_start + nb_frames_temporal_merge) << std::endl;

  int index_merge = 0;

  for (int index_demo = start_index_demo;
      index_demo >= end_index_demo; index_demo--, index_out++) {

    const std::string demo_dir = "rush_3/demo/"; // TODO:
    snprintf(buffer, FILENAME_MAX, std::string(demo_dir + "image_%04d.png").c_str(), index_demo);
    const std::string demo_filename = buffer;
    // std::cout << "demo_filename=" << demo_filename << std::endl;
#if !DEBUG
    vpImageIo::read(I_demo, demo_filename);
    I_out = I_demo;
#endif

    if (index_demo <= index_out_start) {
      const std::string blinding_dir = "rush_3/lights/"; // TODO:
      snprintf(buffer, FILENAME_MAX, std::string(blinding_dir + "image_%04d.png").c_str(), index_blinding);
      const std::string blinding_filename = buffer;
      // std::cout << "blinding_filename=" << blinding_filename << std::endl;
#if !DEBUG
      vpImageIo::read(I_blinding, blinding_filename);
      vpImageTools::resize(I_blinding, I_blinding_resize, vpImageTools::INTERPOLATION_LINEAR);

      const float idx_ratio = index_merge / (float)(end_index_blinding - start_index_blinding);
      const int bb_width = (bb2_width - bb1_width) * idx_ratio + bb1_width;
      const int bb_height = (bb2_height - bb1_height) * idx_ratio + bb1_height;
      // std::cout << "idx_ratio=" << idx_ratio << " ; bb_size=" << bb_width << "x" << bb_height << std::endl;
      // I_smooth = 255;
      // computeSmoothingTable(I_smooth, smooth_h, smooth_w, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor);

      I_smooth2 = 255;
      computeSmoothingTable2(I_smooth2, smooth_h, smooth_w, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor, coeff_a);

      if (0) {
        {
          const std::string smooth_dir = "rush_3/smooth/";
          snprintf(buffer, FILENAME_MAX, std::string(smooth_dir + "image_%04d.png").c_str(), index_out);
          const std::string smooth_filename = buffer;
          vpImageIo::write(I_smooth, smooth_filename);
        }
        {
          const std::string smooth_dir = "rush_3/smooth2/";
          snprintf(buffer, FILENAME_MAX, std::string(smooth_dir + "image_%04d.png").c_str(), index_out);
          const std::string smooth_filename = buffer;
          vpImageIo::write(I_smooth2, smooth_filename);
        }
      }
      // merge(I_demo, I_blinding_resize, I_out, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor, 0.6, 0.4);
      // merge(I_demo, I_blinding_resize, I_out, bb1_top, bb1_left,
      //   bb_width*resize_factor, bb_height*resize_factor, coeff_a, coeff_b, I_smooth);

      mergeSimple(I_demo, I_blinding_resize, I_out, I_smooth2);

      // // TODO:
      // I_out = I_demo;
      // smooth(I_demo, I_out, I_smooth, bb1_top, bb1_left, bb_width*resize_factor, bb_height*resize_factor);
#endif

      index_blinding++;
      index_merge++;
    }
    else if (index_demo <= index_out_start + nb_frames_temporal_merge) {
      float coeff_temporal = (index_out_start + nb_frames_temporal_merge - index_demo) / (float)nb_frames_temporal_merge;
      // std::cout << "index_demo=" << index_demo << " ; coeff_temporal=" << coeff_temporal << std::endl;
      // std::cout << "I_demo=" << I_demo.getCols() << "x" << I_demo.getRows() << std::endl;
      // std::cout << "I_blinding_resize_start=" << I_blinding_resize_start.getCols() << "x" << I_blinding_resize_start.getRows() << std::endl;
      // std::cout << "I_out=" << I_out.getCols() << "x" << I_out.getRows() << std::endl;

#if !DEBUG
      for (int i = bb1_top; i < bb1_height*resize_factor; i++) {
        for (int j = bb1_left; j < bb1_width*resize_factor; j++) {
          vpRGBa a = I_demo[i][j];
          vpRGBa b = I_blinding_resize_start[i][j];
          float coeff_smooth = I_smooth_transition[i][j] / 255.0f;

          float coeff_a_ = coeff_smooth * (1 - coeff_a) + coeff_a;
          float coeff_b_ = 1.0f - coeff_a_;
          vpRGBa c = coeff_a_*a + coeff_b_*b;
          vpRGBa d = (1 - coeff_temporal)*a + coeff_temporal*c;
          I_out[i][j] = d;
        }
      }
#endif
    }

#if !DEBUG
    const std::string output_dir = "rush_3/out/";
    snprintf(buffer, FILENAME_MAX, std::string(output_dir + "image_%04d.png").c_str(), index_out);
    const std::string output_filename = buffer;
    // std::cout << "output_filename=" << output_filename << std::endl;
    vpImageIo::write(I_out, output_filename);
#endif
  }
#endif

  return 0;
}
