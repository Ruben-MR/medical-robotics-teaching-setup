%#codegen
function errorPose = fun_PoseErrorCalulator(desiredTaskspacePose, currentTaskspacePose)

errorPose = zeros(6,1);

desiredRotMat = desiredTaskspacePose(1:3,1:3);
desiredPosition = desiredTaskspacePose(1:3,4);

currentRotMat = currentTaskspacePose(1:3,1:3);
currentPosition = currentTaskspacePose(1:3,4);

errorPos = (desiredPosition - currentPosition);
errorRot = rotMatToRotVec(desiredRotMat*currentRotMat');%  currentRotMat'*[0.0;0.0;0.1];

errorPose = [errorPos; errorRot];

end

