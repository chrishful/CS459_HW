%% Initialization
% HCI Homework 3 Question 2
% Louis Keith and Chris Hickman
% 4-7-22


% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();


% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Keeps track of offscreen movement
moves = 0;
movesOnscreen = 0;

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
        if numPts < 10
                moves = offscreenPrompt(moves);
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);
        

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
         %  videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display tracked points.
          %  videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);

            % Checks the current state of the box
            currentQuadrant = checkBox(bboxPoints);
            disp(currentQuadrant)

            %if the quadrant is the correct quadrant
            if(currentQuadrant == userSpecifiedLocation)
                 % sets my_image to the current frame
                 my_image = videoFrame;
                      % writes my_image to file
                 imwrite(my_image, "this.png")
                 [y, Fs] = audioread('camera.mp3');
                 player = audioplayer(y, Fs);
                 playblocking(player)
                 delete(findall(0))
                 break;
            %otherwise begin to prompt the user     
            elseif (userSpecifiedLocation == "top right") 
                movesOnscreen = topRight(movesOnscreen, currentQuadrant);
            elseif (userSpecifiedLocation == "top left") 
                movesOnscreen = topLeft(movesOnscreen, currentQuadrant);
            elseif (userSpecifiedLocation == "bottom right") 
               movesOnscreen = bottomRight(movesOnscreen, currentQuadrant);
            elseif (userSpecifiedLocation == "bottom left") 
                movesOnscreen = bottomLeft(movesOnscreen, currentQuadrant);
            end

        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    

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
   width = 1280 + 100;
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
function x = topRight(count, position)
    x = count;
    if(position == "top left") %move to the left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
    elseif(position == "center" && mod(count, 2) == 0) %move slightly up
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
        x = count + 1;
    elseif(position == "center" && mod(count, 2) == 1) %move slightly left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
        x = count + 1;
    elseif(position == "bottom right") %move up
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "bottom left") %move up and to the left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    else
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
    end
end

%function to convince a user to move to the top left
function x = topLeft(count, position)
    x = count;
    if(position == "top right") %move to the right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "center" && mod(count, 2) == 0) %move slightly up
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
        x = count + 1;
   elseif(position == "center" && mod(count, 2) == 1) %move slightly right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
        x = count + 1;
    elseif(position == "bottom right") %move up and to the right
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "bottom left") %move up
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
    else  
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
    end
end

%function to convince the user to move to the bottom right
function x = bottomRight(count, position)
    x = count;
    if(position == "top left") %move to the left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
    elseif(position == "center" && mod(count, 2) == 0) %move slightly down
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
        x = count + 1;
    elseif(position == "center" && mod(count, 2) == 1) %move slightly left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
        x = count + 1;
    elseif(position == "top right") %move down
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "bottom left") %move up and to the left
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    else
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
    end
end

%function to convince a user to move to the bottom left
function x = bottomLeft(count, position)
    x = count;
    if(position == "top right") %move to the right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "center" && mod(count, 2) == 0) %move slightly down
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
        x = count + 1;
   elseif(position == "center" && mod(count, 2) == 1) %move slightly right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
        x = count + 1;
    elseif(position == "bottom right") %move right
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)
    elseif(position == "top left") %move up
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player)  
    else  
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
    end
end


function x = offscreenPrompt(moves)
    if moves < 4
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
        x = moves + 1;
    elseif moves < 12
        [y, Fs] = audioread('Left.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
        x = moves + 1;
    elseif moves < 16
        [y, Fs] = audioread('Right.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
        x = moves + 1;
    elseif moves < 20
        [y, Fs] = audioread('Up.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
        x = moves + 1;
    elseif moves < 28
        [y, Fs] = audioread('Down.mp3');
        player = audioplayer(y, Fs);
        playblocking(player) 
        x = moves + 1;
    else 
        disp("please try again")
    end

end




