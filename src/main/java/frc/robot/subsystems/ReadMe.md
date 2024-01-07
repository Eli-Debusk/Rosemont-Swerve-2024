# ROBOT SUBSYSTEMS

## SwerveModule.java
For this specific varient of the Swerve Module, we are using the [MK4I Swerve Modules](https://www.swervedrivespecialties.com/products/mk4i-swerve-module)  
by **Swerve Drive Specailties**. Our chosen devices are: 2 **NEO Brushless Motors**, and  
1 **CTRE-CANCoder**. For our calculations, we use the `SwerveModuleState` class builtin by WPILib

Please note that __resetEncoders()__ and __zeroRelativeEncoderToAbs()__ are not the same. We created a seperate  
function, __zeroRelativeEncoderToAbs()__, to fix a teleOp initialization problem where it would think that  
whatever the pivot's position was on initialization was zero (on the relative encoder), we can fix  
that by setting the relative encoder's position to the absolute encoder's position on initialization.