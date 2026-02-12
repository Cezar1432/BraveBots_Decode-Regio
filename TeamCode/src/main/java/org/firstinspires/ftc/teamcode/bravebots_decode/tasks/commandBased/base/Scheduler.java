package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base;


import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsyncAndConstraints;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsyncAndTime;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToLinearAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToLinearSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToTangentAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToTangentSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.TangentToConstantAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.Wait;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


import java.util.LinkedList;



public class Scheduler {

    Chassis f;




    boolean last= true, current= false, justDone= false;
    public LinkedList<Task> list;

    public Scheduler(){
        list = new LinkedList<>();

    }
    public Scheduler(Chassis f){this.f =f; list = new LinkedList<>();

    }
    public boolean done(){
        return list.isEmpty();
    }
    @Deprecated
    public int getQueueSize(){
        return list.size();
    }

    public Scheduler addChassis(Chassis chassis){
        this.f= chassis;
        return this;
    }




    public Scheduler addTask(Task t){
        list.addLast(t);
        return this;
    }
    public Scheduler addTask(InstantTask t){
        Task task= () -> {
            t.run();
            return true;
        };
        list.addLast(task);
        return this;
    }
    public interface InstantTask{
        void run();
    }
    @Deprecated
    public boolean justDone(){
        return justDone;
    }
    public Scheduler waitSeconds(double sec){

        list.addLast(new Wait(sec));
        return this;
    }
    @Deprecated
    public Scheduler waitSecondsFirst(double sec){
        list.addFirst(new Wait(sec));
        return this;
    }

    public void removeAllTasks(){
        list = new LinkedList<>();
    }
    @Deprecated
    public Scheduler addFirst(Task t){
        list.addFirst(t);
        return this;
    }
    public Scheduler copy(){
        Scheduler copy= new Scheduler();
        for(Task t: list)
            copy.addTask(t);

        return copy;
    }

    public Task getAsTask(){
        Task task;
        class SchedulerAsTask implements Task{

            Scheduler s;
            public SchedulerAsTask(Scheduler s){
                this.s= s.copy();
            }
            @Override
            public boolean Run() {
                s.update();
                return s.done();
            }
        }
        return new SchedulerAsTask(this);
    }
    public void update(){

        if(done()) {
            current= true;
            justDone= !last;
            last= true;
            return;
        }
        current= false;
        last= false;


        Task t= list.getFirst();
        boolean result= t.Run();
        if(result)
            list.removeFirst();

    }
    public Scheduler tangentToConstantAsync(Pose p, double tValue)
    {
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new TangentToConstantAsync());
        return this;
    }
    public Scheduler lineToLinearSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToLinearSync());
        return this;
    }
    public Scheduler lineToLinearAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToLinearAsync());
        return this;
    }

    public Scheduler lineToTangentialSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToTangentSync());
        return this;
    }
    public Scheduler lineToTangentialAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new
                LineToTangentAsync());
        return this;
    }public Scheduler lineToConstantSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToConstantSync(this.f, p));
        return this;
    }public Scheduler lineToConstantAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToConstantAsync(this.f, p));
        return this;
    }
    public Scheduler lineToConstantAsync(Pose p, double time){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToConstantAsyncAndTime(this.f, p, time));
        return this;
    }
    public Scheduler lineToConstantAsync(Pose p, double dist, double rad){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        list.addLast(new LineToConstantAsyncAndConstraints(this.f, p, dist, rad));
        return this;
    }

}
