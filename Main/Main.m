




load("sensorData.mat","allData");


% Inital Values

car1_position_x =  allData(1).ActorPoses(3).Position(1,1);
car1_position_y =  allData(1).ActorPoses(3).Position(1,2);

car1_velocity_x =  allData(1).ActorPoses(3).Velocity(1,1);
car1_velocity_y =  allData(1).ActorPoses(3).Velocity(1,2);


initialState_car1=[car1_position_x;car1_velocity_x;car1_position_y;car1_velocity_y ];



KF_car1= trackingKF('MotionModel', '2D Constant Velocity', 'State', initialState_car1);



next_velocity_car1_x = zeros(11, 1);
next_velocity_car1_y = zeros(11, 1);

next_position_car1_x = zeros(11, 1);
next_position_car1_y = zeros(11, 1);





for i=1:1:9

    current_state_car1 = KF_car1.State;

    next_state_car1 = predict(KF_car1);

    next_velocity_car1_x(i) = next_state_car1(2);
    next_velocity_car1_y(i) = next_state_car1(4);

    next_position_car1_x(i) = next_state_car1(1);
    next_position_car1_y(i) = next_state_car1(3);
       
    meausrments_vector_pos = [allData(i).ActorPoses(3).Position(1,1),allData(i).ActorPoses(3).Position(1,2)];
    
    correct(KF_car1,meausrments_vector_pos);

end





truck_position_x =  allData(1).ActorPoses(2).Position(1,1);
truck_position_y =  allData(1).ActorPoses(2).Position(1,2);


truck_velocity_x =  allData(1).ActorPoses(2).Velocity(1,1);
truck_velocity_y =  allData(1).ActorPoses(2).Velocity(1,2);


initialState_truck=[truck_position_x;truck_velocity_x;truck_position_y;truck_velocity_y];


KF_truck= trackingKF('MotionModel', '2D Constant Velocity', 'State', initialState_truck);


next_velocity_truck_x = zeros(11, 1);
next_velocity_truck_y = zeros(11, 1);

next_position_truck_x = zeros(11, 1);
next_position_truck_y = zeros(11, 1);


for j=1:1:9
    current_state_truck = KF_truck.State;

    next_state_truck = predict(KF_truck);

    next_velocity_truck_x(j) = next_state_truck(2);
    next_velocity_truck_y(j) = next_state_truck(4);

    next_position_truck_x(j) = next_state_truck(1);
    next_position_truck_y(j) = next_state_truck(3);
       

    meausrments_vector_pos_Truck = [allData(j).ActorPoses(2).Position(1,1),allData(j).ActorPoses(2).Position(1,2)];

    correct(KF_truck,meausrments_vector_pos_Truck);
end


New_Position = [allData(1).ActorPoses(1).Position(1,1),allData(1).ActorPoses(1).Position(1,2)] + ([allData(1).ActorPoses(1).Velocity(1,1),allData(1).ActorPoses(1).Velocity(1,2)]* 1.1);





distance_front = sqrt((next_state_truck(1) - New_Position(1))^2 + (next_state_truck(3) - New_Position(2))^2);
distance_left = sqrt((next_state_car1(1) - New_Position(1))^2 + (next_state_car1(3) - New_Position(2))^2);



front = distance_front/30;
left=distance_left/30;

if(front<=30)
    speed=front;
end

ADAS(speed);









