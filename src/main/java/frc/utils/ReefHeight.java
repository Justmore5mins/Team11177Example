package frc.utils;

public enum ReefHeight {
    L0(10),
    L1(20),
    L2(30),
    L3(40);

    private double height;
    ReefHeight(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
