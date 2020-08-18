#define LAYER_1_SIZE 20
#define LAYER_2_SIZE 20
#define LAYER_3_SIZE 3

float layer_1[LAYER_1_SIZE] = {0};
float layer_2[LAYER_2_SIZE] = {0};
float layer_3[LAYER_3_SIZE] = {0};


std::vector<float> float_matmul(std::vector<float> input, std::vector<float> weights){
    uint i = 0;
    int output_size = (int)(weights.size()/input.size());
    std::vector<float> output(output_size,0);

    while (i< weights.size()){
        for (uint k=0; k< input.size(); k++ ){
            for (int j = 0; j < output_size; j++) {
                output[j] += + weights[i]*input[k];
                i++;
            }
        }
    }
    return output;
}

std::vector<float> float_bias_add(std::vector<float> input, std::vector<float> weights){
    std::vector<float> output(input.size(),0);
    for(uint i = 0; i<input.size(); i++){
        output[i] = input[i]+weights[i];
    }
    return output;
}

std::vector<float> float_relu(std::vector<float> input){
    std::vector<float> output(input.size(),0);
    for (uint i = 0 ; i< input.size(); i++){
        if(input[i]>0){
            output[i] = input[i];
        }
    }
    return output;
}


int argmax_float(std::vector<float> input){
    float max = input[0];
    int max_ind = 0;
    for (uint i = 1; i< input.size(); i++){
        if (input[i]>max){
            max = input[i];
            max_ind = i;
        }
    }
    return max_ind;
}

/**
 * @param input (float vector): input vector into the network
 * @param params (float vector): vector with all weight and biasadd terms
 * @param shape (float vector): vector with shape of all layers in network, including input/output layers
*/

std::vector<float> trim_weights(std::vector<float> weights,int start, int stop)
{
    std::vector<float> output;
    for (int i = start; i < stop; i++)
    {
        output.push_back(weights[i]);
    }
    return output;
}

int float_inference(std::vector<float> input, std::vector<float> params, std::vector<float> shape){


    //layer 1
    std::vector<float> tensor, local_weights;
    int weight_start, weight_stop;
    
    // input layer
    weight_start = 0;
    weight_stop = shape[0]*shape[1];
    tensor = float_matmul(input,trim_weights(params,weight_start,weight_stop));
    weight_start = weight_stop;
    weight_stop = weight_start+shape[1];
    tensor = float_bias_add(tensor,trim_weights(params,weight_start,weight_stop));
    tensor = float_relu(tensor);

    // hidden layers
    for (uint j = 1; j<(shape.size()-2);j++)
    {
        weight_start = weight_stop;
        weight_stop = weight_start+shape[j]*shape[j+1];
        tensor = float_matmul(tensor,trim_weights(params,weight_start,weight_stop));
        weight_start = weight_stop;
        weight_stop = weight_start+shape[j+1];
        tensor = float_bias_add(tensor,trim_weights(params,weight_start,weight_stop));
        tensor = float_relu(tensor);  
    }

    // output layer
    weight_start = weight_stop;
    weight_stop = weight_start+shape[shape.size()-2]*shape[shape.size()-1];
    tensor = float_matmul(tensor,trim_weights(params,weight_start,weight_stop));
    weight_start = weight_stop;
    weight_stop = weight_start+shape[shape.size()-1];
    tensor = float_bias_add(tensor,trim_weights(params,weight_start,weight_stop));
    return argmax_float(tensor);

}