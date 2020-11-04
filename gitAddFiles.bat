echo on
echo .
echo git add files to commmit ... 
echo allways the same files to add, so best put it in a batch file, 
echo needed since this somewhat strange ´staged-to-commit´ feature had been added
echo .
echo 
git add ST/tempix-03/*
git add ST/STM32F767ZI_Nucleo/*
echo do not git add ST/BSP/STM32F767ZI_Nucleo/stm32f7xx_hal_conf.h

git add .gitignore

git add gitAddFiles.bat
git add checkOriginMaster.bat


git status
