<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<!-- [![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url] -->



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!-- <a href="https://github.com/github_username/repo_name">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

<h3 align="center">igcclib</h3>

  <p align="center">
   Igamenovoer's Common Code Library, my personal C++ code snippets and utilities.
    <!-- <br />
    <a href="https://igamenovoer.github.io/PeiDocker"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">Examples</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a> -->
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<!-- <details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details> -->



<!-- ABOUT THE PROJECT -->
## About The Project

My code snippets and utilities for C++ projects, through the years. 

I tried my best to make the code as bug-free as possible, but no guarantee, use at your own risk.

<!-- _For details, please refer to the [Documentation](https://igamenovoer.github.io/PeiDocker/)_ -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ### Built With

* [![Next][Next.js]][Next-url]
* [![React][React.js]][React-url]
* [![Vue][Vue.js]][Vue-url]
* [![Angular][Angular.io]][Angular-url]
* [![Svelte][Svelte.dev]][Svelte-url]
* [![Laravel][Laravel.com]][Laravel-url]
* [![Bootstrap][Bootstrap.com]][Bootstrap-url]
* [![JQuery][JQuery.com]][JQuery-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- GETTING STARTED -->
## Getting Started

Download this project and use CMake to build the library. Then, just like normal CMake projects, you can link the library to your project.

Like this:

```cmake
find_package(igcclib COMPONENTS core vision geometry)
target_link_libraries(your_target igcclib::core igcclib::vision igcclib::geometry)
```

The library is divided into several components:
- `core`: core utilities, headers only, depends on [Eigen3](https://eigen.tuxfamily.org/)
- `vision`: computer vision utilities, depends on [OpenCV](https://opencv.org/)
- `geometry`: geometry, mainly based on [CGAL](https://www.cgal.org/)
- `math`: math utilities
- `io`: input/output utilities, serialization is provided by [cereal](https://uscilab.github.io/cereal/) and [msgpack](https://msgpack.org/)
- `crypto`: cryptography utilities, wrapped from [openssl](https://www.openssl.org/)
- `graph`: graph algorithms, wrapped from [boost::graph](https://www.boost.org/doc/libs/1_76_0/libs/graph/doc/index.html)
- `python_boostpy`: python bindings, wrapped from [boost::python](https://www.boost.org/doc/libs/1_76_0/libs/python/doc/html/index.html)
- `python_pybind11`: python bindings, wrapped from [pybind11](https://pybind11.readthedocs.io/en/stable/)
- `visualization`: visualization utilities
- `device`: device utilities, to access different kinds of cameras, sensors, etc.