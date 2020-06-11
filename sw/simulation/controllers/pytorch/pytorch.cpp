#include "pytorch.h"
#include "draw.h"
#include <torch/torch.h>

void pytorch::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
}
