# gnc-sim WASM entry. Only configured under Emscripten (see top-level CMakeLists). Produced via
# scripts/build-wasm.sh, which copies gncsim.js/.wasm into web/public/wasm/.
add_executable(gncsim_wasm gncsim_wasm.cpp)
target_link_libraries(gncsim_wasm PRIVATE gncsim_core)

set_target_properties(gncsim_wasm PROPERTIES OUTPUT_NAME "gncsim")

# embind + ES6 module output + growable heap; exported runtime methods for JS glue.
target_link_options(gncsim_wasm PRIVATE
  "--bind"
  "-sMODULARIZE=1"
  "-sEXPORT_ES6=1"
  "-sENVIRONMENT=web,worker,node"
  "-sALLOW_MEMORY_GROWTH=1"
  "-sEXPORT_NAME=createGncSim"
  "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
)
