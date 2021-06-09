echo on
echo .
echo git add files to commmit ... 
echo allways the same files to add, so best put it in a batch file, 
echo needed since this somewhat strange ´staged-to-commit´ feature had been added
echo .
echo 
git add tempix-03/ST/tempix-03/*
git add tempix-03/ST/STM32F756ZG_Nucleo/*
git add tempix-03/Micrium/*
git add tempix-03/docs/*
git add tempix-03/tempixTerminal/*
git add tempix-03/.gitignore
git add tempix-03/.project

git add tempix-03/ST/BSP/*
git add tempix-03/tempixTerminal/*

git add tempixActuator/*
git add tempixActuator

git add commonResources/*

git add gitAddFiles.bat
git add checkOriginMaster.bat
git add .gitignore

git status
