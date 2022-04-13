% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [50 50 [frameSize(2), frameSize(1)]+10]);

runLoop = true;
numPts = 0;
frameCount = 0;
userSpecifiedLocation = "";
timer = timer();



%asks for user's input, ensures that it is one of the four required
while true
    prompt = "specify top left, top right, bottom left, or bottom right";
    userSpecifiedLocation = input(prompt);
    if (userSpecifiedLocation == "top left" || userSpecifiedLocation == "top right" || userSpecifiedLocation == "bottom left" || userSpecifiedLocation == "bottom right")
         break;
    end
end

while runLoop
    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));
       

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;
            

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);
        if numPts < 10
                offscreenPrompt();
        end

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, inlierIdx] = estimateGeometricTransform2D(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
            oldInliers    = oldInliers(inlierIdx, :);
            visiblePoints = visiblePoints(inlierIdx, :);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the face being tracked.
           % videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display tracked points.
          %  videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);

            % Checks the current state of the box
            currentQuadrant = checkBox(bboxPoints);

            %if the quadrant is the correct quadrant
            if(currentQuadrant == userSpecifiedLocation)
                 % sets my_image to the current frame
                 my_image = videoFrame;
                      % writes my_image to file
                 imwrite(my_image, "this.png")
                 delete(findall(0))
                 break;
            %otherwise begin to prompt the user     
            elseif (userSpecifiedLocation == "top right") 
                topRight(currentQuadrant)
            elseif (userSpecifiedLocation == "top left") 
                topLeft(currentQuadrant)
            elseif (userSpecifiedLocation == "bottom right") 
                bottomRight(currentQuadrant)
            elseif (userSpecifiedLocation == "bottom left") 
                bottomLeft(currentQuadrant)
            end

        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    
    %sets a timer for 5 seconds
    %pause(2)

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end



% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);



% Checks the four corners of a box and determines the
% quadrant it is in
function x = checkBox(bboxPoints)    %width and height of photograph + some error
   width = 1280 + 200;
   height = 720 + 100;


   if isempty(bboxPoints)
       x = "offscreen";
   elseif bboxPoints(1, 1) < width/2 && bboxPoints(1,2) < height/2 && bboxPoints(2, 1) < width/2 && bboxPoints(2,2) < height/2 && bboxPoints(3, 1) < width/2 && bboxPoints(3,2) < height/2 && bboxPoints(4, 1) < width/2 && bboxPoints(4,2) < height/2
       x = "top left";
   elseif (bboxPoints(1, 1) > width/2 && bboxPoints(1,2) > height/2 && bboxPoints(2, 1) > width/2 && bboxPoints(2,2) > height/2 && bboxPoints(3, 1) > width/2 && bboxPoints(3,2) > height/2 && bboxPoints(4, 1) > width/2 && bboxPoints(4,2) > height/2)
       x = "bottom right";
   elseif (bboxPoints(1, 1) > width/2 && bboxPoints(1,2) < height/2 && bboxPoints(2, 1) > width/2 && bboxPoints(2,2) < height/2 && bboxPoints(3, 1) > width/2 && bboxPoints(3,2) < height/2 && bboxPoints(4, 1) > width/2 && bboxPoints(4,2) < height/2)
       x = "top right";
   elseif (bboxPoints(1, 1) < width/2 && bboxPoints(1,2) > height/2 && bboxPoints(2, 1) < width/2 && bboxPoints(2,2) > height/2 && bboxPoints(3, 1) < width/2 && bboxPoints(3,2) > height/2 && bboxPoints(4, 1) < width/2 && bboxPoints(4,2) > height/2)
       x = "bottom left";
   else 
       x = "center";
   end

end


%function to convince the user to move to the top right
function topRight(position)
    if(position == "top left") %move to the left
    elseif(position == "center") %move slightly to the left
    elseif(position == "bottom right") %move up
    elseif(position == "bottom left") %move up and to the left
    elseif(position == "top right") %do nothing
    else  %do the offscreen stuff?????
    end
end

%function to convince a user to move to the top left
function topLeft(position)
    if(position == "top right") %move to the right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        play(player)  
    elseif(position == "center") %move slightly to the right
    elseif(position == "bottom right") 
        %move up and to the right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        play(player)
    elseif(position == "bottom left") %move up 
    elseif(position == "top left") %do nothing
    else  %do the offscreen stuff?????
    end
end

%function to convince a user to move to the bottom right
function bottomRight(position)
    if(position == "top left") %move down and to the left
    elseif(position == "center") %move slightly to the left
    elseif(position == "top right") %move down
    elseif(position == "bottom left") %move to the left
    elseif(position == "bottom right") %do nothing
    else  %do the offscreen stuff?????
    end
end

%function to convince a user to move to the bottom left
function bottomLeft(position)
    if(position == "top left") %move down
    elseif(position == "center") %move slightly to the right
    elseif(position == "bottom right") %move to the right
    elseif(position == "top right") %move down and to the right
    elseif(position == "bottom left") %do nothing
    else  %do the offscreen stuff?????
    end
end

function offscreenPrompt()
    disp('get onscreen.')
end




