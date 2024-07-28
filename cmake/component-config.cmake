# add dependecy check
include(CMakeFindDependencyMacro)

# foreach library in DependencyList, find_dependency(library)
foreach(dep ${DependencyList})
  find_dependency(${dep})
endforeach()

# given ComponentName, include the target file

