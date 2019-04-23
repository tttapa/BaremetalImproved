// Original: Cleanup-Pieter/Code-Generators/Controllers/include/attitude-controller.h

// ! Siel: this should be a symbolic link, so if we change the MATLAB code,
// !       then this will update as well.
// !
// !       Syntax if working directory == .../attitude/include/
// !       Syntax Windows: mklink "..\relative\path\file.hpp" "shortcut.hpp"
// !       Syntax Linux: ln -s "../relative/path/file.hpp" "shortcut.hpp"
// !
// !       Problem: we're not in the same project, so we can't use relative path to Codegen
// !                we're editing on different computers, so we can't use absolute path to Codegen
// !
// !       Solution 1: Move Cleanup-Pieter to BaremetalImproved and use relative path
// !       Solution 2: Move BaremetalImproved to GitLab and use relative path
// !       Solution 3: Remove [...]-controller.hpp and just link it in Vivado SDK
// !
// !       This is Pieter's GitHub and it's Pieter's directory in GitLab, so it's his decision
// !

