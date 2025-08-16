using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using HelixToolkit.Wpf;

namespace Project
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        Model3DGroup robot;
        Model3DGroup Base;
        Model3DGroup link1;
        Model3DGroup link2;
        Model3DGroup link3;
        Model3DGroup end;

        Model3DGroup Object;
        Model3D cube;

        ModelImporter importer;

        Point3D joint1_p;
        Point3D joint2_p;
        Point3D joint3_p;        
        
        Double link1_len;
        Double link2_len;
        Double link3_len;
        Double link0_len;


        Vector3D ee_global;
        Vector3D ee_global_x;
        Vector3D ee_global_y;
        Vector3D ee_global_z;

        Vector3D cube_global;
        Vector3D cube_global_x;
        Vector3D cube_global_y;
        Vector3D cube_global_z;

        Vector3D ee_orientation_u;
        Vector3D ee_orientation_v;
        Vector3D ee_orientation_w;

        Vector3D cube_orientation_u;
        Vector3D cube_orientation_v;
        Vector3D cube_orientation_w;

        Point3D tcp;
        Boolean cube_picked;

        public MainWindow()
        {
            InitializeComponent();
            robot = new Model3DGroup();
            importer = new ModelImporter();
            Base = importer.Load(@"asset/Base.obj");
            link1 = importer.Load(@"asset/link1.obj");
            link2 = importer.Load(@"asset/link2.obj");
            link3 = importer.Load(@"asset/link3.obj");
            end = importer.Load(@"asset/end.obj");
            robot.Children.Add(Base);
            Base.Children.Add(link1);
            link1.Children.Add(link2);
            link2.Children.Add(link3);
            link3.Children.Add(end);
            model1.Content = robot;

            joint1_p = new Point3D(0, 0, 3.5);
            joint2_p = new Point3D(0, 6, 3.5);
            joint3_p = new Point3D(0, 10, 3.5);
            tcp = new Point3D(0, 13.75, 3.5);

            link0_len = Math.Sqrt(Math.Pow(joint1_p.X, 2) + Math.Pow(joint1_p.Y, 2) + Math.Pow(joint1_p.Z, 2));
            link1_len = Math.Sqrt(Math.Pow((joint1_p.X-joint2_p.X),2)+ Math.Pow((joint1_p.Y - joint2_p.Y), 2) + Math.Pow((joint1_p.Z - joint2_p.Z), 2));
            link2_len = Math.Sqrt(Math.Pow((joint2_p.X - joint3_p.X), 2) + Math.Pow((joint2_p.Y - joint3_p.Y), 2) + Math.Pow((joint2_p.Z - joint3_p.Z), 2));
            link3_len = Math.Sqrt(Math.Pow((joint3_p.X - tcp.X), 2) + Math.Pow((joint3_p.Y - tcp.Y), 2) + Math.Pow((joint3_p.Z - tcp.Z), 2));

            cube_x_show.Text = $"{link0_len}";
            cube_y_show.Text = $"{link1_len}";
            cube_z_show.Text = $"{link2_len} , {link3_len}";           

            joint1_show.Text = $"Joint 1: {joint1.Value.ToString("F3")}";
            joint2_show.Text = $"Joint 2: {joint2.Value.ToString("F3")}";
            joint3_show.Text = $"Joint 3: {joint3.Value.ToString("F3")}";
            base_show.Text = $"Base : {base_slider.Value.ToString("F3")}";    

            px_e.Text = $"{tcp.X.ToString("F2")}";
            py_e.Text = $"{tcp.Y.ToString("F2")}";
            pz_e.Text = $"{tcp.Z.ToString("F2")}";

            cube_picked = false;
            pick.IsEnabled = false;
            drop.IsEnabled = false;

        }

        private void end_effector_data_update()
        {
            //Position Update
            Double j1 = (Math.PI/180)*joint1.Value;
            Double j2 = (Math.PI / 180) * joint2.Value;
            Double j3 = (Math.PI / 180) * joint3.Value;
            Double b = (Math.PI /180) * base_slider.Value;
            Double local_x = link1_len*Math.Cos(j1) + link2_len*Math.Cos(j1+j2) + link3_len* Math.Cos(j1+j2+j3);
            Double local_y = link0_len + link1_len*Math.Sin(j1) + link2_len*Math.Sin(j1+j2) + link3_len*Math.Sin(j1+j2+j3);
            Double r = Math.Sqrt((local_x * local_x) + (local_y * local_y));
            ee_global.Z = local_y;
            ee_global.X = r * Math.Sin(b);
            ee_global.Y = r * Math.Cos(b);
            tcp.X = ee_global.X;
            tcp.Y = ee_global.Y;
            tcp.Z = ee_global.Z;

            ee_global_x = new Vector3D(tcp.X, 0, 0);
            ee_global_y = new Vector3D(0, tcp.Y, 0);
            ee_global_z = new Vector3D(0 , 0, tcp.Z);

            //Orientation update
            ee_orientation_u.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), ee_global_x);
            ee_orientation_u.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), ee_global_x);
            ee_orientation_u.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), ee_global_x);

            ee_orientation_v.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), ee_global_y);
            ee_orientation_v.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), ee_global_y);
            ee_orientation_v.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), ee_global_y);

            ee_orientation_w.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), ee_global_z);
            ee_orientation_w.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), ee_global_z);
            ee_orientation_w.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), ee_global_z);



        }
        private void end_effector_data_display()
        {
            px_e.Text = $"{ee_global.X.ToString("F2")}";
            py_e.Text = $"{ee_global.Y.ToString("F2")}";
            pz_e.Text = $"{ee_global.Z.ToString("F2")}";

            ux_e.Text = $"{ee_orientation_u.X.ToString("F2")}";
            uy_e.Text = $"{ee_orientation_u.Y.ToString("F2")}";
            uz_e.Text = $"{ee_orientation_u.Z.ToString("F2")}";

            vx_e.Text = $"{ee_orientation_v.X.ToString("F2")}";
            vy_e.Text = $"{ee_orientation_v.Y.ToString("F2")}";
            vz_e.Text = $"{ee_orientation_v.Z.ToString("F2")}";

            wx_e.Text = $"{ee_orientation_w.X.ToString("F2")}";
            wy_e.Text = $"{ee_orientation_w.Y.ToString("F2")}";
            wz_e.Text = $"{ee_orientation_w.Z.ToString("F2")}";

        }
        private void move_base(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), base_slider.Value));
            robot.Transform = rotateTransform3D;
            base_show.Text = $"Base : {base_slider.Value.ToString("F3")}";

            end_effector_data_update();
            end_effector_data_display();
        }

        private void move_joint3(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint3.Value));
            rotateTransform3D.CenterX = joint3_p.X;
            rotateTransform3D.CenterY = joint3_p.Y;
            rotateTransform3D.CenterZ = joint3_p.Z;
            end.Transform = rotateTransform3D;
            joint3_show.Text = $"Joint 3: {joint3.Value.ToString("F3")}";

            end_effector_data_update();
            end_effector_data_display();
        }

        private void move_joint2(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint2.Value));
            rotateTransform3D.CenterX = joint2_p.X;
            rotateTransform3D.CenterY = joint2_p.Y;
            rotateTransform3D.CenterZ = joint2_p.Z;
            link3.Transform = rotateTransform3D;
            joint2_show.Text = $"Joint 2: {joint2.Value.ToString("F3")}";

            end_effector_data_update();
            end_effector_data_display();
        }

        private void move_joint1(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint1.Value));
            rotateTransform3D.CenterX = joint1_p.X;
            rotateTransform3D.CenterY = joint1_p.Y;
            rotateTransform3D.CenterZ = joint1_p.Z;
            link2.Transform = rotateTransform3D;
            joint1_show.Text = $"Joint 1: {joint1.Value.ToString("F3")}";

            end_effector_data_update();
            end_effector_data_display();
        }

        private void cube_data_update()
        {
            //Position Update            
            cube_global.X = cube_x.Value;
            cube_global.Y = cube_y.Value;
            cube_global.Z = cube_z.Value;

            cube_global_x = new Vector3D(cube_global.X, 0, 0);
            cube_global_y = new Vector3D(0, cube_global.Y, 0);
            cube_global_z = new Vector3D(0, 0, cube_global.Z);

            //Orientation update
            cube_orientation_u.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), cube_global_x);
            cube_orientation_u.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), cube_global_x);
            cube_orientation_u.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), cube_global_x);

            cube_orientation_v.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), cube_global_y);
            cube_orientation_v.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), cube_global_y);
            cube_orientation_v.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), cube_global_y);

            cube_orientation_w.X = Vector3D.AngleBetween(new Vector3D(1, 0, 0), cube_global_z);
            cube_orientation_w.Y = Vector3D.AngleBetween(new Vector3D(0, 1, 0), cube_global_z);
            cube_orientation_w.Z = Vector3D.AngleBetween(new Vector3D(0, 0, 1), cube_global_z);
        }

        private void cube_data_display()
        {
            px_c.Text = $"{cube_global.X.ToString("F2")}";
            py_c.Text = $"{cube_global.Y.ToString("F2")}";
            pz_c.Text = $"{cube_global.Z.ToString("F2")}";

            ux_c.Text = $"{cube_orientation_u.X.ToString("F2")}";
            uy_c.Text = $"{cube_orientation_u.Y.ToString("F2")}";
            uz_c.Text = $"{cube_orientation_u.Z.ToString("F2")}";

            vx_c.Text = $"{cube_orientation_v.X.ToString("F2")}";
            vy_c.Text = $"{cube_orientation_v.Y.ToString("F2")}";
            vz_c.Text = $"{cube_orientation_v.Z.ToString("F2")}";

            wx_c.Text = $"{cube_orientation_w.X.ToString("F2")}";
            wy_c.Text = $"{cube_orientation_w.Y.ToString("F2")}";
            wz_c.Text = $"{cube_orientation_w.Z.ToString("F2")}";
        }

        private void add_cube(object sender, RoutedEventArgs e)
        {
            Object = new Model3DGroup();
            cube = importer.Load(@"asset/cube.obj");
            Object.Children.Add(cube);
            model2.Content = Object;

            cube_add.IsEnabled = false;
            cube_remove.IsEnabled = true;
            cube_x.IsEnabled = true;
            cube_y.IsEnabled = true;
            cube_z.IsEnabled = true;
            cube_y.Value = 10;
            pick.IsEnabled = true;
            MessageBox.Show("Cube added to the workspace!!");
            cube_x_show.Text = $"X: {cube_x.Value.ToString("F3")}";
            cube_y_show.Text = $"Y: {cube_y.Value.ToString("F3")}";
            cube_z_show.Text = $"Z: {cube_z.Value.ToString("F3")}";            

            px_c.Text = $"{cube_x.Value.ToString("F2")}";
            py_c.Text = $"{cube_y.Value.ToString("F2")}";
            pz_c.Text = $"{cube_z.Value.ToString("F2")}";
        }

        private void remove_cube(object sender, RoutedEventArgs e)
        {
            if (!cube_picked)
            {
                model2.Content = null;
                cube_add.IsEnabled = true;
                cube_remove.IsEnabled = false;
                cube_y.Value = 0;
                cube_x.Value = 0;
                cube_z.Value = 0;
                cube_x.IsEnabled = false;
                cube_y.IsEnabled = false;
                cube_z.IsEnabled = false;
                MessageBox.Show("Cube removed from the workspace!!");
                cube_x_show.Text = null;
                cube_y_show.Text = null;
                cube_z_show.Text = null;

                ux_c.Text = "0";
                uy_c.Text = "0";
                uz_c.Text = "0";
                vx_c.Text = "0";
                vy_c.Text = "0";
                vz_c.Text = "0";
                wx_c.Text = "0";
                wy_c.Text = "0";
                wz_c.Text = "0";
                px_c.Text = "0";
                py_c.Text = "0";
                pz_c.Text = "0";
            }
            else
            { 
                drop_cube(null, new RoutedEventArgs());
                remove_cube(null, new RoutedEventArgs());
            }

        }

        private void move_cube_x(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;
            cube_x_show.Text = $"X: {cube_x.Value.ToString("F3")}";
            px_c.Text = $"{cube_x.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void move_cube_y(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;
            cube_y_show.Text = $"Y: {cube_y.Value.ToString("F3")}";            
            py_c.Text = $"{cube_y.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void move_cube_z(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;
            cube_z_show.Text = $"Z: {cube_z.Value.ToString("F3")}";           
            pz_c.Text = $"{cube_z.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void pick_cube(object sender, RoutedEventArgs e)
        {
            if (!cube_picked)
            {
                model2.Content = null;                
                cube_picked = true;
                Object.Children.Remove(cube);
                end.Children.Add(cube);

                cube_y.Value = 14;
                cube_z.Value = 3;

                pick.IsEnabled = false;
                drop.IsEnabled = true;
                cube_x.IsEnabled = false;
                cube_y.IsEnabled = false;
                cube_z.IsEnabled = false;
            }
        }

        private void drop_cube(object sender, RoutedEventArgs e)
        {
            if (cube_picked)
            {
                cube_picked = false;
                model2.Content = Object;
                end.Children.Remove(cube);
                Object = new Model3DGroup();
                Object.Children.Add(cube);
                model2.Content = Object;
                RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), -180));                
                Object.Transform = rotateTransform3D;                
                cube_x.Value = tcp.X;
                cube_y.Value = tcp.Y;
                cube_z.Value = 0;
                pick.IsEnabled = true;
                drop.IsEnabled = false;
                cube_x.IsEnabled = true;
                cube_y.IsEnabled = true;
                cube_z.IsEnabled = true;
                cube_x.Value = tcp.X;
                cube_y.Value = tcp.Y;
                cube_z.Value = 0;
            }
        }

        private void reset_robot(object sender, RoutedEventArgs e)
        {
            joint1.Value = 21;
            joint2.Value = -37;
            joint3.Value = -66;
            base_slider.Value = 0;

        }
    }
}
