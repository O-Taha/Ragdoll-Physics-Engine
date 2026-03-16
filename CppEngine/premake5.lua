workspace "RagdollEngine"
    architecture "x64"
    configurations { "Debug", "Release" }

project "RagdollEngine"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"

    targetdir "bin/%{cfg.buildcfg}"
    objdir "bin-int/%{cfg.buildcfg}"

    files { "src/**.cpp" }

    includedirs {
        "/usr/local/include"
    }

    libdirs {
        "/usr/local/lib"
    }

    links {
        "raylib",
        "GL",
        "m",
        "pthread",
        "dl",
        "rt",
        "X11"
    }

    filter "configurations:Debug"
        symbols "On"

    filter "configurations:Release"
        optimize "On"