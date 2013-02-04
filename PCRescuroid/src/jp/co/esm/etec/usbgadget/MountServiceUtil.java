package jp.co.esm.etec.usbgadget;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

import android.content.Context;
import android.os.IBinder;
import android.util.Log;

/**
 * {@link MountService}のユーティリティクラスです。
 */
public class MountServiceUtil {
	
	/**
	 * {@link MountService}を取得します。
	 * 
	 * @param context コンテキスト
	 * @return {@link MountService}
	 */
	public static MountService getMountService(Context context) {
		//Object mountService = context.getSystemService("mount");
		Object mountService = _getMountService(context);
		MountServiceInvocationHandler handler = new MountServiceInvocationHandler(mountService);
		return (MountService)Proxy.newProxyInstance(context.getClassLoader(), new Class[]{MountService.class}, handler);
	}
	
	private static Object _getMountService(Context context) {
		Class<?> cls;
		try {
			cls = Class.forName("android.os.ServiceManager");
			Method method = cls.getMethod("getService", new Class[]{String.class});
			Object binder = method.invoke(cls, "mount");
			Class<?> imountserviceStubCls = Class.forName("android.os.IMountService$Stub");
			Method asInterface = imountserviceStubCls.getMethod("asInterface", new Class[]{IBinder.class});
			return asInterface.invoke(imountserviceStubCls, binder);
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SecurityException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (NoSuchMethodException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	/**
	 * {@link MountService}のメソッド呼び出しに応じてcom.android.server.MountServiceのメソッドを呼び出す{@link InvocationHandler}です。
	 * ISOファイルのマウント機能はカスタマイズされたcom.android.server.MountServiceが提供しますが
	 * com.android.server.MountServiceは非公開APIであるため、直接そのメソッドを呼び出すことができません。
	 * そのため、 ISOファイルのマウント操作に必要なメソッドを定義した{@link MountService}越しにcom.android.server.MountServiceのメソッドを呼び出すことで
	 * 必要な機能を提供します。
	 */
	private static class MountServiceInvocationHandler implements InvocationHandler {
		/**
		 * com.android.server.MountServiceインスタンス
		 */
		private Object mountService;
		
		/**
		 * 処理を転送するcom.android.server.MountServiceインスタンスを指定して{@link MountServiceInvocationHandler}を作成します。
		 * 
		 * @param mountService com.android.server.MountServiceインスタンス
		 */
		public MountServiceInvocationHandler(Object mountService) {
			this.mountService = mountService;
		}
		
		/**
		 * {@inheritDoc}
		 */
		@Override
		public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
			try {
				//
//				Method[] methods = mountService.getClass().getMethods();
//				for (Method m : methods) {
//					Log.d("MountServiceUtil", m.getName());
//				}
				Method serviceMethod = mountService.getClass().getMethod(method.getName(), method.getParameterTypes());
				return serviceMethod.invoke(mountService, args);
			} catch (NoSuchMethodException e) {
				throw new UnsupportedOperationException(mountService.getClass().getName() + "." + method.getName() + "() method is not found.", e);
			}
		}
	}
}
