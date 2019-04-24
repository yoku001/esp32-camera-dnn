set(COMPONENT_SRCDIRS .
  nnabla-c-runtime/src/functions/implements/array 
  nnabla-c-runtime/src/functions/implements/logical 
  nnabla-c-runtime/src/functions/implements/quantization 
  nnabla-c-runtime/src/functions/implements/math 
  nnabla-c-runtime/src/functions/implements/activation 
  nnabla-c-runtime/src/functions/implements/arithmetic 
  nnabla-c-runtime/src/functions/implements/neural_network/affine 
  nnabla-c-runtime/src/functions/implements/neural_network/convolution 
  nnabla-c-runtime/src/functions/implements/neural_network 
  nnabla-c-runtime/src/functions/implements/normalization 
  nnabla-c-runtime/src/functions/implements/reduction 
  nnabla-c-runtime/src/functions/implements/stochasticity 
  nnabla-c-runtime/src/functions/utilities
)
set(COMPONENT_ADD_INCLUDEDIRS . nnabla-c-runtime/src/functions)
set(COMPONENT_PRIV_INCLUDEDIRS nnabla-c-runtime/include)

register_component()
